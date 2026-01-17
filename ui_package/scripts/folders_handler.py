#!/usr/bin/env python3
"""
folders_handler.py (PRODUCT-READY + MAPPING/NAV MODE SWITCH)

Adds:
- Mapping mode: PAUSE Nav2 lifecycle (localization + optional navigation) + start slam_toolbox
- Save map: stop slam_toolbox + RESUME Nav2 + load map
- Cancel mapping: stop slam_toolbox + RESUME Nav2 (so you never get stuck)
- If node restarts and slam_toolbox is still running: best-effort "force stop" support
"""

import os
import re
import csv
import json
import time
import yaml
import shutil
import signal
import subprocess
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_msgs.srv import LoadMap
from nav2_msgs.srv import ManageLifecycleNodes  # Nav2 lifecycle manager service
from ament_index_python.packages import get_package_share_directory

from ui_package.msg import ArrayPoseStampedWithCovariance


_SAFE_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")


class UIFoldersHandler(Node):
    # Nav2 ManageLifecycleNodes command constants (Humble+)
    STARTUP = 0
    PAUSE = 1
    RESUME = 2
    RESET = 3
    SHUTDOWN = 4

    def __init__(self):
        super().__init__("ui_folders_handler")

        # QoS for UI messages so late subscribers (GUI) get last messages
        self.qos_ui = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # -------- Parameters --------
        self.declare_parameter("data_dir", os.path.join(os.path.expanduser("~"), ".ros", "amr_gui_data"))
        self.declare_parameter("odom_topic", "/odom")

        # Launch strings
        self.declare_parameter("mappingLaunch", "slam_toolbox online_async_launch.py")
        self.declare_parameter("navigationLaunch", "ebot_nav2 ebot_bringup_launch.py")

        # Demo defaults
        self.declare_parameter("demo_group", "nan")
        self.declare_parameter("demo_map", "nan")

        # Lifecycle switching behavior
        self.declare_parameter("enable_lifecycle_switch", True)
        self.declare_parameter("pause_navigation_during_mapping", True)

        # Common Nav2 service names (configurable)
        self.declare_parameter("localization_manage_srv", "/lifecycle_manager_localization/manage_nodes")
        self.declare_parameter("navigation_manage_srv", "/lifecycle_manager_navigation/manage_nodes")
        # Fallback if your bringup uses a single lifecycle manager
        self.declare_parameter("single_manage_srv", "/lifecycle_manager/manage_nodes")

        # Best-effort force kill if slam_toolbox was started outside / handler restarted
        self.declare_parameter("allow_force_kill_slam", True)

        self.data_dir = self.get_parameter("data_dir").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.mappingCmd = self.get_parameter("mappingLaunch").value
        self.navigationCmd = self.get_parameter("navigationLaunch").value
        self.demo_group = self.get_parameter("demo_group").value
        self.demo_map = self.get_parameter("demo_map").value

        self.enable_lifecycle_switch = bool(self.get_parameter("enable_lifecycle_switch").value)
        self.pause_navigation_during_mapping = bool(self.get_parameter("pause_navigation_during_mapping").value)

        self.loc_manage_srv = self.get_parameter("localization_manage_srv").value
        self.nav_manage_srv = self.get_parameter("navigation_manage_srv").value
        self.single_manage_srv = self.get_parameter("single_manage_srv").value

        self.allow_force_kill_slam = bool(self.get_parameter("allow_force_kill_slam").value)

        # -------- Runtime state --------
        self.WPs: List[str] = []
        self.waypoints: List[Tuple[PoseWithCovarianceStamped, float]] = []
        self.position = None

        # Mapping process handle (only if we started it)
        self.slam_proc = None

        # Mode flags
        self.mapping_active = False

        # -------- Storage paths (WRITABLE) --------
        self.maps_folder = os.path.join(self.data_dir, "maps")
        self.routes_folder = os.path.join(self.data_dir, "paths")
        self.current_files = os.path.join(self.data_dir, "current_map_route.yaml")

        self._ensure_storage_dirs()
        self._ensure_demo_map()
        self._sanitize_current_files()

        # -------- ROS: clients --------
        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")

        # Nav2 lifecycle manager service clients (optional)
        self.loc_lc_client = self.create_client(ManageLifecycleNodes, self.loc_manage_srv)
        self.nav_lc_client = self.create_client(ManageLifecycleNodes, self.nav_manage_srv)
        self.single_lc_client = self.create_client(ManageLifecycleNodes, self.single_manage_srv)

        # -------- ROS: subs --------
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.ui_sub = self.create_subscription(String, "ui_operation", self.ui_callback, 10)
        self.new_wp_sub = self.create_subscription(PoseWithCovarianceStamped, "/new_way_point", self.new_way_point_callback, 10)
        self.nav_data_sub = self.create_subscription(Empty, "/nav_data_req", self.nav_data_callback, 10)
        self.wp_req_sub = self.create_subscription(Empty, "WP_req", self.WP_req_callback, 10)

        # -------- ROS: pubs --------
        self.ui_pub = self.create_publisher(String, "ui_message", self.qos_ui)
        self.poseArray_pub = self.create_publisher(ArrayPoseStampedWithCovariance, "WayPoints_topic", 10)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.nav_data_pub = self.create_publisher(String, "nav_data_resp", 10)

        self.get_logger().info("✅ UI folders handler started (product storage + mode switching)")
        self._ui_info("STARTED", "UI folders handler started", data_dir=self.data_dir)

        # If slam_toolbox already running (page refresh / node restart), warn and tell user to cancel mapping
        if self._detect_slam_toolbox_running():
            self.mapping_active = True
            self._ui_warn(
                "SLAM_DETECTED",
                "⚠️ slam_toolbox appears to be running already (maybe page refresh / node restart). "
                "Use 'cancel_mapping' to exit mapping mode safely.",
            )

        # publish initial structure once
        self._publish_nav_structure()

    # =========================
    # UI Message helpers (JSON)
    # =========================
    def _ui_emit(self, level: str, code: str, message: str, **details):
        payload = {
            "level": level,
            "code": code,
            "message": message,
            "details": details or {},
            "ts": time.time(),
        }
        try:
            self.ui_pub.publish(String(data=json.dumps(payload)))
        except Exception as e:
            self.get_logger().error(f"Failed to publish ui_message: {e}")

    def _ui_info(self, code: str, message: str, **details):
        self._ui_emit("INFO", code, message, **details)

    def _ui_warn(self, code: str, message: str, **details):
        self._ui_emit("WARN", code, message, **details)

    def _ui_error(self, code: str, message: str, **details):
        self._ui_emit("ERROR", code, message, **details)

    # =========================
    # Storage helpers
    # =========================
    def _ensure_storage_dirs(self):
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.maps_folder, exist_ok=True)
        os.makedirs(self.routes_folder, exist_ok=True)

        if not os.path.exists(self.current_files):
            with open(self.current_files, "w") as f:
                yaml.safe_dump({"map_file": "", "route_file": ""}, f)

    def _ensure_demo_map(self):
        group = self.demo_group
        mp = self.demo_map

        demo_group_dir = os.path.join(self.maps_folder, group)
        os.makedirs(demo_group_dir, exist_ok=True)

        demo_yaml = os.path.join(demo_group_dir, f"{mp}.yaml")
        demo_img_pgm = os.path.join(demo_group_dir, f"{mp}.pgm")
        demo_img_png = os.path.join(demo_group_dir, f"{mp}.png")

        if os.path.isfile(demo_yaml) and (os.path.isfile(demo_img_pgm) or os.path.isfile(demo_img_png)):
            os.makedirs(os.path.join(self.routes_folder, group, mp), exist_ok=True)
            return

        try:
            share_dir = get_package_share_directory("ui_package")
            candidates = [
                os.path.join(share_dir, "demo", "maps", group, mp),
                os.path.join(share_dir, "maps", group, mp),
                os.path.join(share_dir, "demo_maps", group, mp),
            ]

            copied = False
            for base in candidates:
                yaml_src = f"{base}.yaml"
                pgm_src = f"{base}.pgm"
                png_src = f"{base}.png"

                if os.path.isfile(yaml_src) and (os.path.isfile(pgm_src) or os.path.isfile(png_src)):
                    shutil.copy2(yaml_src, demo_yaml)
                    if os.path.isfile(pgm_src):
                        shutil.copy2(pgm_src, demo_img_pgm)
                    else:
                        shutil.copy2(png_src, demo_img_png)
                    copied = True
                    break

            if copied:
                os.makedirs(os.path.join(self.routes_folder, group, mp), exist_ok=True)
                self._ui_info("DEMO_READY", "Demo map copied into data_dir.", group=group, map=mp)
            else:
                self._ui_warn(
                    "DEMO_MISSING",
                    "Demo map assets not found in package share. Create or copy a map into data_dir/maps.",
                    expected_demo=f"{group}/{mp}",
                    data_dir=self.data_dir,
                )
        except Exception as e:
            self._ui_warn("DEMO_COPY_FAIL", "Failed while preparing demo map.", err=str(e))

    def _sanitize_current_files(self):
        data = self.get_cur_files()
        changed = False

        map_file = data.get("map_file", "") or ""
        route_file = data.get("route_file", "") or ""

        if map_file and not os.path.isfile(map_file):
            self.get_logger().warn(f"Stale map_file in current yaml: {map_file} -> reset")
            data["map_file"] = ""
            changed = True

        if route_file and not os.path.isfile(route_file):
            self.get_logger().warn(f"Stale route_file in current yaml: {route_file} -> reset")
            data["route_file"] = ""
            changed = True

        if changed:
            with open(self.current_files, "w") as f:
                yaml.safe_dump(data, f)
            self._ui_warn("ACTIVE_RESET", "Stale active map/route reset because file was missing.")

        if not data.get("map_file"):
            demo_yaml = os.path.join(self.maps_folder, self.demo_group, f"{self.demo_map}.yaml")
            if os.path.isfile(demo_yaml):
                self.set_cur_map(os.path.join(self.maps_folder, self.demo_group, self.demo_map))
                self.set_cur_route("")
                self._ui_info("ACTIVE_SET_DEMO", "Active map set to demo.", group=self.demo_group, map=self.demo_map)

    def _safe_name(self, name: str) -> bool:
        return bool(name) and bool(_SAFE_NAME_RE.match(name))

    def _group_dir(self, group: str) -> str:
        return os.path.join(self.maps_folder, group)

    def _map_base(self, group: str, mp: str) -> str:
        return os.path.join(self.maps_folder, group, mp)

    def _route_dir(self, group: str, mp: str) -> str:
        return os.path.join(self.routes_folder, group, mp)

    def _route_file(self, group: str, mp: str, route: str) -> str:
        return os.path.join(self._route_dir(group, mp), f"{route}.csv")

    # =========================
    # Current files get/set
    # =========================
    def get_cur_files(self) -> Dict[str, str]:
        try:
            with open(self.current_files, "r") as f:
                data = yaml.safe_load(f) or {}
            return {"map_file": data.get("map_file", "") or "", "route_file": data.get("route_file", "") or ""}
        except Exception as e:
            self.get_logger().error(f"Error reading current yaml: {e}")
            return {"map_file": "", "route_file": ""}

    def set_cur_map(self, map_base_path: str):
        data = self.get_cur_files()
        data["map_file"] = f"{map_base_path}.yaml" if map_base_path else ""
        with open(self.current_files, "w") as f:
            yaml.safe_dump(data, f)

    def set_cur_route(self, route_base_path: str):
        data = self.get_cur_files()
        data["route_file"] = f"{route_base_path}.csv" if route_base_path else ""
        with open(self.current_files, "w") as f:
            yaml.safe_dump(data, f)

    # =========================
    # ROS callbacks
    # =========================
    def odom_callback(self, msg: Odometry):
        try:
            self.position = msg.pose.pose
        except Exception as e:
            self._ui_warn("ODOM_PARSE_FAIL", "Failed to parse odom.", err=str(e))

    def nav_data_callback(self, _msg: Empty):
        self._publish_nav_structure()

    def WP_req_callback(self, _msg: Empty):
        try:
            self.read_wp_safe()
            self.poseArray_pub.publish(self._convert_pose_cov_list(self.waypoints))
        except Exception as e:
            self._ui_error("WP_REQ_FAIL", "Failed handling WP_req.", err=str(e))
            self.poseArray_pub.publish(ArrayPoseStampedWithCovariance())

    def new_way_point_callback(self, msg: PoseWithCovarianceStamped):
        try:
            line = (
                f"{msg.pose.pose.position.x},"
                f"{msg.pose.pose.position.y},"
                f"{msg.pose.pose.position.z},"
                f"{msg.pose.pose.orientation.x},"
                f"{msg.pose.pose.orientation.y},"
                f"{msg.pose.pose.orientation.z},"
                f"{msg.pose.pose.orientation.w},"
                f"{msg.pose.covariance[0]},"
                f"{msg.pose.covariance[1]},"
                f"{msg.pose.covariance[2]}"
            )
            if line not in self.WPs:
                self.WPs.append(line)

            self._ui_info("WP_ADDED", f"{len(self.WPs)} waypoint(s) in buffer.", count=len(self.WPs))
        except Exception as e:
            self._ui_error("WP_ADD_FAIL", "Failed to add waypoint.", err=str(e))

    # =========================
    # Conversions
    # =========================
    def _convert_pose_cov_list(self, waypoints: List[Tuple[PoseWithCovarianceStamped, float]]) -> ArrayPoseStampedWithCovariance:
        arr = ArrayPoseStampedWithCovariance()
        try:
            for pose_cov, _purpose in waypoints:
                arr.poses.append(pose_cov)
        except Exception:
            return ArrayPoseStampedWithCovariance()
        return arr

    # =========================
    # Nav2 lifecycle helpers
    # =========================
    def _call_manage_nodes(self, client, srv_name: str, command: int, timeout_sec: float = 2.0) -> bool:
        if not self.enable_lifecycle_switch:
            return True

        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False

            req = ManageLifecycleNodes.Request()
            req.command = int(command)
            fut = client.call_async(req)

            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
            if not fut.done():
                return False

            resp = fut.result()
            return bool(getattr(resp, "success", False))
        except Exception:
            return False

    def _pause_nav2_for_mapping(self):
        """
        Pause localization (AMCL + map_server typically) and optionally navigation servers.
        This prevents /map from being published by map_server while slam_toolbox runs.
        """
        if not self.enable_lifecycle_switch:
            return

        ok_any = False

        # Prefer split managers (common in bringup)
        ok_loc = self._call_manage_nodes(self.loc_lc_client, self.loc_manage_srv, self.PAUSE)
        ok_nav = True
        if self.pause_navigation_during_mapping:
            ok_nav = self._call_manage_nodes(self.nav_lc_client, self.nav_manage_srv, self.PAUSE)

        if ok_loc or ok_nav:
            ok_any = True

        # Fallback: single manager
        if not ok_any:
            ok_single = self._call_manage_nodes(self.single_lc_client, self.single_manage_srv, self.PAUSE)
            ok_any = ok_single

        if ok_any:
            self._ui_info("NAV2_PAUSED", "Nav2 lifecycle paused for mapping (AMCL/map_server inactive).")
        else:
            self._ui_warn(
                "NAV2_PAUSE_FAIL",
                "⚠️ Could not pause Nav2 lifecycle managers. Mapping will still run, but you may see /map conflicts.",
                loc_srv=self.loc_manage_srv,
                nav_srv=self.nav_manage_srv,
                single_srv=self.single_manage_srv,
            )

    def _resume_nav2_after_mapping(self):
        """
        Resume localization + navigation lifecycle managers after slam_toolbox is stopped.
        """
        if not self.enable_lifecycle_switch:
            return

        ok_any = False

        ok_loc = self._call_manage_nodes(self.loc_lc_client, self.loc_manage_srv, self.RESUME)
        ok_nav = True
        if self.pause_navigation_during_mapping:
            ok_nav = self._call_manage_nodes(self.nav_lc_client, self.nav_manage_srv, self.RESUME)

        if ok_loc or ok_nav:
            ok_any = True

        if not ok_any:
            ok_single = self._call_manage_nodes(self.single_lc_client, self.single_manage_srv, self.RESUME)
            ok_any = ok_single

        if ok_any:
            self._ui_info("NAV2_RESUMED", "Nav2 lifecycle resumed (navigation mode).")
        else:
            self._ui_warn(
                "NAV2_RESUME_FAIL",
                "⚠️ Could not resume Nav2 lifecycle managers. You may need to restart navigation bringup.",
                loc_srv=self.loc_manage_srv,
                nav_srv=self.nav_manage_srv,
                single_srv=self.single_manage_srv,
            )

    # =========================
    # slam_toolbox process helpers
    # =========================
    def _detect_slam_toolbox_running(self) -> bool:
        """
        Detect slam_toolbox nodes by ROS graph (best-effort).
        """
        try:
            names = [n for (n, _ns) in self.get_node_names_and_namespaces()]
            for n in names:
                if "slam_toolbox" in n:
                    return True
        except Exception:
            pass
        return False

    def _force_kill_slam_toolbox(self):
        """
        Best-effort force-stop slam_toolbox if we lost the Popen handle (node restarted).
        This is intentionally limited to slam_toolbox-related patterns.
        """
        if not self.allow_force_kill_slam:
            self._ui_warn("SLAM_FORCE_DISABLED", "Force-kill slam_toolbox is disabled by parameter.")
            return

        try:
            # Try common patterns (limited scope)
            subprocess.call("pkill -f slam_toolbox >/dev/null 2>&1", shell=True)
            subprocess.call("pkill -f async_slam_toolbox_node >/dev/null 2>&1", shell=True)
            subprocess.call("pkill -f sync_slam_toolbox_node >/dev/null 2>&1", shell=True)
            self._ui_warn("SLAM_FORCE_KILL", "⚠️ Forced stop requested for slam_toolbox (best-effort).")
        except Exception as e:
            self._ui_warn("SLAM_FORCE_KILL_FAIL", "Force stop failed.", err=str(e))

    def _stop_slam_toolbox(self):
        """
        Stop slam_toolbox if we started it.
        If we didn't start it but detect it's running, allow best-effort force kill via pkill.
        """
        try:
            # If we have a process handle, stop cleanly
            if self.slam_proc is not None:
                if self.slam_proc.poll() is not None:
                    self.slam_proc = None
                    return

                pid = self.slam_proc.pid
                try:
                    pgid = os.getpgid(pid)
                except Exception:
                    pgid = None

                self._ui_info("SLAM_STOP_REQ", "Stopping slam_toolbox...", pid=pid)

                # Graceful Ctrl+C
                if pgid is not None:
                    os.killpg(pgid, signal.SIGINT)
                else:
                    os.kill(pid, signal.SIGINT)

                try:
                    self.slam_proc.wait(timeout=3.0)
                    self._ui_info("SLAM_STOPPED", "slam_toolbox stopped.", pid=pid)
                    self.slam_proc = None
                    return
                except Exception:
                    pass

                # SIGTERM
                if pgid is not None:
                    os.killpg(pgid, signal.SIGTERM)
                else:
                    os.kill(pid, signal.SIGTERM)

                try:
                    self.slam_proc.wait(timeout=2.0)
                    self._ui_info("SLAM_STOPPED", "slam_toolbox stopped (SIGTERM).", pid=pid)
                    self.slam_proc = None
                    return
                except Exception:
                    pass

                # SIGKILL
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    os.kill(pid, signal.SIGKILL)

                self._ui_warn("SLAM_KILLED", "slam_toolbox killed (SIGKILL).", pid=pid)
                self.slam_proc = None
                return

            # No handle: if slam seems running, do best-effort force stop
            if self._detect_slam_toolbox_running():
                self._force_kill_slam_toolbox()

        except Exception as e:
            self._ui_warn("SLAM_STOP_FAIL", "Failed stopping slam_toolbox.", err=str(e))

    def _start_slam_toolbox(self):
        """
        Start slam_toolbox, storing a Popen handle.
        """
        try:
            # Stop any existing slam we started
            self._stop_slam_toolbox()

            self.slam_proc = subprocess.Popen(
                f"ros2 launch {self.mappingCmd}",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
            )
            self._ui_info("SLAM_STARTED", "slam_toolbox started.", pid=self.slam_proc.pid)
        except Exception as e:
            self._ui_error("SLAM_START_FAIL", "Failed to start slam_toolbox.", err=str(e))
            self.slam_proc = None

    # =========================
    # Read route safely
    # =========================
    def read_wp_safe(self):
        self.waypoints.clear()
        cur = self.get_cur_files()
        route_file = cur.get("route_file", "")

        if not route_file:
            self._ui_warn("NO_ROUTE", "No route selected.")
            return

        if not os.path.isfile(route_file):
            self._ui_warn("ROUTE_MISSING", "Route file missing. Resetting active route.", route_file=route_file)
            self.set_cur_route("")
            self._publish_nav_structure()
            return

        bad_rows = 0
        try:
            with open(route_file, "r") as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    if len(row) < 11:
                        bad_rows += 1
                        continue
                    try:
                        p = PoseWithCovarianceStamped()
                        p.header.frame_id = "map"
                        p.pose.pose.position.x = float(row[0])
                        p.pose.pose.position.y = float(row[1])
                        p.pose.pose.position.z = float(row[2])
                        p.pose.pose.orientation.x = float(row[3])
                        p.pose.pose.orientation.y = float(row[4])
                        p.pose.pose.orientation.z = float(row[5])
                        p.pose.pose.orientation.w = float(row[6])
                        p.pose.covariance[0] = float(row[7])
                        p.pose.covariance[1] = float(row[8])
                        p.pose.covariance[2] = float(row[9])
                        purpose = float(row[10])
                        self.waypoints.append((p, purpose))
                    except Exception:
                        bad_rows += 1
                        continue
        except Exception as e:
            self._ui_error("ROUTE_READ_FAIL", "Failed to read route CSV.", route_file=route_file, err=str(e))
            return

        if bad_rows:
            self._ui_warn("ROUTE_BAD_ROWS", "Some waypoint rows were invalid and were skipped.", bad_rows=bad_rows)

        if not self.waypoints:
            self._ui_warn("ROUTE_EMPTY", "The waypoint queue is empty.")

    # =========================
    # Structure listing
    # =========================
    def get_paths_json(self) -> str:
        try:
            structure = []
            os.makedirs(self.routes_folder, exist_ok=True)

            for group in sorted(os.listdir(self.routes_folder)):
                gdir = os.path.join(self.routes_folder, group)
                if not os.path.isdir(gdir):
                    continue

                maps_list = []
                for mp in sorted(os.listdir(gdir)):
                    mdir = os.path.join(gdir, mp)
                    if not os.path.isdir(mdir):
                        continue
                    routes = sorted([f for f in os.listdir(mdir) if f.lower().endswith(".csv")])
                    maps_list.append({mp: routes})

                structure.append({group: maps_list})

            cur = self.get_cur_files()
            active_group = "Null"
            active_map = "Null"
            active_route = "Null"

            if cur.get("route_file"):
                parts = cur["route_file"].replace("\\", "/").split("/")
                if len(parts) >= 3:
                    active_group = parts[-3]
                    active_map = parts[-2]
                    active_route = os.path.splitext(parts[-1])[0]
            elif cur.get("map_file"):
                parts = cur["map_file"].replace("\\", "/").split("/")
                if len(parts) >= 2:
                    active_group = parts[-2]
                    active_map = os.path.splitext(parts[-1])[0]
                    active_route = "Null"

            resp = {"structure": structure, "active_files": {"group": active_group, "map": active_map, "route": active_route}}
            return json.dumps(resp)
        except Exception as e:
            self._ui_error("STRUCT_FAIL", "Failed to build nav structure.", err=str(e))
            return json.dumps({"structure": [], "active_files": {"group": "Null", "map": "Null", "route": "Null"}})

    def _publish_nav_structure(self):
        self.nav_data_pub.publish(String(data=self.get_paths_json()))

    # =========================
    # Map server interaction
    # =========================
    def _load_map(self, map_yaml_file: str):
        if not os.path.isfile(map_yaml_file):
            self._ui_error("MAP_YAML_MISSING", "Map yaml not found.", map_yaml=map_yaml_file)
            return

        if not self.load_map_client.wait_for_service(timeout_sec=2.0):
            self._ui_error("MAP_SERVER_DOWN", "map_server/load_map service not available. Is Nav2 running?")
            return

        req = LoadMap.Request()
        req.map_url = map_yaml_file

        self._ui_info("MAP_LOAD_REQ", "Loading map...", map_yaml=map_yaml_file)
        fut = self.load_map_client.call_async(req)
        fut.add_done_callback(lambda f: self._load_map_done(f, map_yaml_file))

    def _load_map_done(self, future, map_yaml_file: str):
        try:
            resp = future.result()
            ok = getattr(resp, "result", 0) == 0
            if ok:
                self._ui_info("MAP_LOADED", "Map loaded successfully.", map_yaml=map_yaml_file)
            else:
                self._ui_error("MAP_LOAD_FAIL", "Map load failed (service returned error).", map_yaml=map_yaml_file, result=int(getattr(resp, "result", -1)))
        except Exception as e:
            self._ui_error("MAP_LOAD_EXC", "Exception while loading map.", map_yaml=map_yaml_file, err=str(e))

    # =========================
    # Mapping / Navigation mode actions
    # =========================
    def build_map_func(self):
        """
        Enter mapping mode:
        - PAUSE Nav2 lifecycle managers (AMCL/map_server inactive)
        - Start slam_toolbox
        """
        try:
            self.poseArray_pub.publish(ArrayPoseStampedWithCovariance())
            self._ui_info("MAPPING_START", "Mapping started. Drive robot around to build map.")

            self._pause_nav2_for_mapping()
            self._start_slam_toolbox()

            self.mapping_active = True
        except Exception as e:
            self._ui_error("MAPPING_FAIL", "Failed to start mapping.", err=str(e))

    def cancel_mapping_func(self):
        """
        Exit mapping mode WITHOUT saving:
        - Stop slam_toolbox
        - RESUME Nav2 lifecycle managers
        """
        try:
            self._ui_info("MAPPING_CANCEL", "Cancel mapping requested. Exiting mapping mode (no save).")
            self._stop_slam_toolbox()
            self._resume_nav2_after_mapping()
            self.mapping_active = False
        except Exception as e:
            self._ui_error("MAPPING_CANCEL_FAIL", "Failed to cancel mapping.", err=str(e))

    def save_map_func(self):
        """
        Save map and exit mapping mode:
        - Save map to data_dir
        - Stop slam_toolbox
        - Resume Nav2
        - Load saved map via map_server
        """
        try:
            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            if not (self._safe_name(group) and self._safe_name(mp)):
                self._ui_error("BAD_NAME", "Invalid group/map name. Use letters/numbers/_- only.", group=group, map=mp)
                return

            os.makedirs(self._group_dir(group), exist_ok=True)
            os.makedirs(self._route_dir(group, mp), exist_ok=True)

            map_path_base = self._map_base(group, mp)
            self._ui_info("MAP_SAVING", "Saving map...", group=group, map=mp)

            cmd = f"ros2 run nav2_map_server map_saver_cli -f {map_path_base}"
            rc = os.system(cmd)
            if rc != 0:
                self._ui_error("MAP_SAVE_FAIL", "Map save failed. You can 'cancel_mapping' to exit mapping mode.", rc=rc)
                return

            # Set active map; reset route
            self.set_cur_map(map_path_base)
            self.set_cur_route("")
            self.WPs.clear()

            # Stop SLAM (so it stops publishing /map)
            self._stop_slam_toolbox()

            # Resume Nav2 lifecycle (so map_server/amcl become active again)
            self._resume_nav2_after_mapping()
            self.mapping_active = False

            # Publish initialpose from current odom if available
            if self.position is not None:
                p = PoseWithCovarianceStamped()
                p.header.frame_id = "map"
                p.pose.pose = self.position
                self.set_pose_pub.publish(p)

            # Load the saved map now that map_server is active again
            self._load_map(f"{map_path_base}.yaml")

            self._ui_info("MAP_SAVED", "Map saved. Navigation mode restored.", group=group, map=mp)
            self._publish_nav_structure()

        except Exception as e:
            self._ui_error("SAVE_MAP_EXC", "Exception in save_map.", err=str(e))

    def change_map_func(self):
        """
        Switch to a saved map (navigation mode).
        Ensure SLAM is stopped + Nav2 resumed.
        """
        try:
            # If mapping is running (even if page refreshed), exit mapping mode first
            if self.mapping_active or self._detect_slam_toolbox_running():
                self._ui_warn("MAPPING_ACTIVE", "⚠️ Mapping is active. Stopping SLAM and restoring navigation mode.")
                self._stop_slam_toolbox()
                self._resume_nav2_after_mapping()
                self.mapping_active = False

            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            if not (self._safe_name(group) and self._safe_name(mp)):
                self._ui_error("BAD_NAME", "Invalid group/map name.", group=group, map=mp)
                return

            map_yaml = f"{self._map_base(group, mp)}.yaml"
            if not os.path.isfile(map_yaml):
                self._ui_error("MAP_NOT_FOUND", "Selected map does not exist in data_dir.", map_yaml=map_yaml)
                return

            self.set_cur_map(self._map_base(group, mp))
            self.set_cur_route("")
            self.WPs.clear()

            self._load_map(map_yaml)

            self._ui_info("MAP_CHANGED", "Map selected. Now select a route (if any).", group=group, map=mp)
            self.WP_req_callback(Empty())
            self._publish_nav_structure()

        except Exception as e:
            self._ui_error("CHANGE_MAP_EXC", "Exception in change_map.", err=str(e))

    # =========================
    # Group/Map/Route ops (unchanged)
    # =========================
    def create_group_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            if not self._safe_name(group):
                self._ui_error("BAD_GROUP", "Invalid group name.", group=group)
                return
            os.makedirs(self._group_dir(group), exist_ok=True)
            os.makedirs(os.path.join(self.routes_folder, group), exist_ok=True)
            self._ui_info("GROUP_CREATED", "Group created.", group=group)
            self._publish_nav_structure()
        except Exception as e:
            self._ui_error("GROUP_CREATE_EXC", "Exception creating group.", err=str(e))

    def rename_group_func(self):
        try:
            old = self.dict_cmd.get("group_old", "")
            new = self.dict_cmd.get("group_new", "")
            if not (self._safe_name(old) and self._safe_name(new)):
                self._ui_error("BAD_GROUP", "Invalid group name(s).", old=old, new=new)
                return

            old_maps = os.path.join(self.maps_folder, old)
            new_maps = os.path.join(self.maps_folder, new)
            old_routes = os.path.join(self.routes_folder, old)
            new_routes = os.path.join(self.routes_folder, new)

            if not os.path.isdir(old_maps):
                self._ui_error("GROUP_NOT_FOUND", "Old group does not exist.", group=old)
                return
            if os.path.exists(new_maps) or os.path.exists(new_routes):
                self._ui_error("GROUP_EXISTS", "New group name already exists.", group=new)
                return

            os.rename(old_maps, new_maps)
            if os.path.isdir(old_routes):
                os.rename(old_routes, new_routes)
            else:
                os.makedirs(new_routes, exist_ok=True)

            cur = self.get_cur_files()
            if cur.get("map_file") and f"/{old}/" in cur["map_file"].replace("\\", "/"):
                cur["map_file"] = cur["map_file"].replace(f"/{old}/", f"/{new}/")
            if cur.get("route_file") and f"/{old}/" in cur["route_file"].replace("\\", "/"):
                cur["route_file"] = cur["route_file"].replace(f"/{old}/", f"/{new}/")
            with open(self.current_files, "w") as f:
                yaml.safe_dump(cur, f)

            self._ui_info("GROUP_RENAMED", "Group renamed.", old=old, new=new)
            self._publish_nav_structure()
        except Exception as e:
            self._ui_error("GROUP_RENAME_EXC", "Exception renaming group.", err=str(e))

    def rename_map_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            old = self.dict_cmd.get("map_old", "")
            new = self.dict_cmd.get("map_new", "")

            if not (self._safe_name(group) and self._safe_name(old) and self._safe_name(new)):
                self._ui_error("BAD_NAME", "Invalid group/map name(s).", group=group, old=old, new=new)
                return

            old_base = self._map_base(group, old)
            new_base = self._map_base(group, new)
            old_yaml = f"{old_base}.yaml"
            if not os.path.isfile(old_yaml):
                self._ui_error("MAP_NOT_FOUND", "Map does not exist.", map_yaml=old_yaml)
                return

            try:
                with open(old_yaml, "r") as f:
                    y = yaml.safe_load(f) or {}
            except Exception:
                y = None

            old_pgm = f"{old_base}.pgm"
            old_png = f"{old_base}.png"
            new_pgm = f"{new_base}.pgm"
            new_png = f"{new_base}.png"

            if y is not None and "image" in y:
                if os.path.isfile(old_pgm):
                    y["image"] = os.path.basename(new_pgm)
                elif os.path.isfile(old_png):
                    y["image"] = os.path.basename(new_png)
                with open(old_yaml, "w") as f:
                    yaml.safe_dump(y, f)

            os.rename(old_yaml, f"{new_base}.yaml")
            if os.path.isfile(old_pgm):
                os.rename(old_pgm, new_pgm)
            elif os.path.isfile(old_png):
                os.rename(old_png, new_png)

            old_rdir = self._route_dir(group, old)
            new_rdir = self._route_dir(group, new)
            if os.path.isdir(old_rdir):
                os.rename(old_rdir, new_rdir)
            else:
                os.makedirs(new_rdir, exist_ok=True)

            cur = self.get_cur_files()
            if cur.get("map_file") == f"{old_base}.yaml":
                self.set_cur_map(new_base)
            if cur.get("route_file") and f"/{group}/{old}/" in cur["route_file"].replace("\\", "/"):
                cur["route_file"] = cur["route_file"].replace(f"/{group}/{old}/", f"/{group}/{new}/")
                with open(self.current_files, "w") as f:
                    yaml.safe_dump(cur, f)

            self._ui_info("MAP_RENAMED", "Map renamed.", group=group, old=old, new=new)
            self._publish_nav_structure()
        except Exception as e:
            self._ui_error("MAP_RENAME_EXC", "Exception renaming map.", err=str(e))

    def delete_map_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            if not (self._safe_name(group) and self._safe_name(mp)):
                self._ui_error("BAD_NAME", "Invalid group/map name.", group=group, map=mp)
                return

            base = self._map_base(group, mp)
            yaml_file = f"{base}.yaml"
            pgm_file = f"{base}.pgm"
            png_file = f"{base}.png"

            for fpath in [yaml_file, pgm_file, png_file]:
                if os.path.isfile(fpath):
                    os.remove(fpath)

            rdir = self._route_dir(group, mp)
            if os.path.isdir(rdir):
                shutil.rmtree(rdir)

            cur = self.get_cur_files()
            if cur.get("map_file") == yaml_file:
                self.set_cur_map("")
                self.set_cur_route("")

            self.WPs.clear()
            self._ui_info("MAP_DELETED", "Map deleted.", group=group, map=mp)
            self.WP_req_callback(Empty())
            self._publish_nav_structure()
        except Exception as e:
            self._ui_error("MAP_DELETE_EXC", "Exception deleting map.", err=str(e))

    def delete_group_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            if not self._safe_name(group):
                self._ui_error("BAD_GROUP", "Invalid group name.", group=group)
                return

            g_maps = os.path.join(self.maps_folder, group)
            g_routes = os.path.join(self.routes_folder, group)

            if os.path.isdir(g_maps):
                shutil.rmtree(g_maps)
            if os.path.isdir(g_routes):
                shutil.rmtree(g_routes)

            cur = self.get_cur_files()
            if cur.get("map_file") and f"/{group}/" in cur["map_file"].replace("\\", "/"):
                self.set_cur_map("")
                self.set_cur_route("")

            self.WPs.clear()
            self._ui_info("GROUP_DELETED", "Group deleted.", group=group)
            self.WP_req_callback(Empty())
            self._publish_nav_structure()
        except Exception as e:
            self._ui_error("GROUP_DELETE_EXC", "Exception deleting group.", err=str(e))

    def clear_route_func(self):
        self.WPs.clear()
        self._ui_info("WP_CLEARED", "Waypoints cleared. Set new points on the map.")

    def save_route_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            route = self.dict_cmd.get("route", "")

            if not (self._safe_name(group) and self._safe_name(mp) and self._safe_name(route)):
                self._ui_error("BAD_NAME", "Invalid group/map/route name(s).", group=group, map=mp, route=route)
                return

            if not self.WPs:
                self._ui_warn("NO_WP_BUFFER", "No waypoints in buffer to save.")
                return

            os.makedirs(self._route_dir(group, mp), exist_ok=True)

            route_file = self._route_file(group, mp, route)
            with open(route_file, "w") as f:
                for wp in self.WPs:
                    f.write(wp + ",1\n")

            self.set_cur_route(os.path.join(self._route_dir(group, mp), route))
            count = len(self.WPs)
            self.WPs.clear()

            self._ui_info("ROUTE_SAVED", "Route saved.", group=group, map=mp, route=route, count=count)
            self._publish_nav_structure()

        except Exception as e:
            self._ui_error("ROUTE_SAVE_EXC", "Exception saving route.", err=str(e))

    def delete_route_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            route = self.dict_cmd.get("route", "")

            if not (self._safe_name(group) and self._safe_name(mp) and self._safe_name(route)):
                self._ui_error("BAD_NAME", "Invalid group/map/route name(s).", group=group, map=mp, route=route)
                return

            file_path = self._route_file(group, mp, route)
            if os.path.isfile(file_path):
                os.remove(file_path)

            cur = self.get_cur_files()
            if cur.get("route_file") == file_path:
                self.set_cur_route("")

            self._ui_info("ROUTE_DELETED", "Route deleted.", group=group, map=mp, route=route)
            self.WP_req_callback(Empty())
            self._publish_nav_structure()

        except Exception as e:
            self._ui_error("ROUTE_DELETE_EXC", "Exception deleting route.", err=str(e))

    def change_route_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            route = self.dict_cmd.get("route", "")

            if not (self._safe_name(group) and self._safe_name(mp) and self._safe_name(route)):
                self._ui_error("BAD_NAME", "Invalid group/map/route name(s).", group=group, map=mp, route=route)
                return

            file_path = self._route_file(group, mp, route)
            if not os.path.isfile(file_path):
                self._ui_error("ROUTE_NOT_FOUND", "Route file does not exist.", route_file=file_path)
                return

            self.set_cur_route(os.path.join(self._route_dir(group, mp), route))
            self.read_wp_safe()
            self.poseArray_pub.publish(self._convert_pose_cov_list(self.waypoints))
            self._ui_info("ROUTE_CHANGED", "Route selected.", group=group, map=mp, route=route)
            self._publish_nav_structure()

        except Exception as e:
            self._ui_error("ROUTE_CHANGE_EXC", "Exception changing route.", err=str(e))

    def rename_route_func(self):
        try:
            group = self.dict_cmd.get("group", "")
            mp = self.dict_cmd.get("map", "")
            old = self.dict_cmd.get("route_old", "")
            new = self.dict_cmd.get("route_new", "")

            if not (self._safe_name(group) and self._safe_name(mp) and self._safe_name(old) and self._safe_name(new)):
                self._ui_error("BAD_NAME", "Invalid group/map/route name(s).", group=group, map=mp, old=old, new=new)
                return

            old_file = self._route_file(group, mp, old)
            new_file = self._route_file(group, mp, new)

            if not os.path.isfile(old_file):
                self._ui_error("ROUTE_NOT_FOUND", "Old route does not exist.", route_file=old_file)
                return
            if os.path.exists(new_file):
                self._ui_error("ROUTE_EXISTS", "New route name already exists.", route_file=new_file)
                return

            os.rename(old_file, new_file)

            cur = self.get_cur_files()
            if cur.get("route_file") == old_file:
                self.set_cur_route(os.path.join(self._route_dir(group, mp), new))

            self._ui_info("ROUTE_RENAMED", "Route renamed.", group=group, map=mp, old=old, new=new)
            self._publish_nav_structure()
        except Exception as e:
            self._ui_error("ROUTE_RENAME_EXC", "Exception renaming route.", err=str(e))

    # =========================
    # UI command dispatcher
    # =========================
    def ui_callback(self, msg: String):
        try:
            raw = (msg.data or "").strip()
            if not raw:
                self._ui_warn("EMPTY_CMD", "Empty ui_operation received.")
                return

            parts = raw.split("/", 1)
            cmd = parts[0].strip()

            self.dict_cmd = {}
            if len(parts) == 2 and parts[1].strip():
                try:
                    self.dict_cmd = json.loads(parts[1])
                except Exception as e:
                    self._ui_error("BAD_JSON", "Invalid JSON payload in ui_operation.", err=str(e), raw=raw)
                    return

            self.get_logger().info(f"ui_operation: {cmd}")

            if cmd == "build_map":
                self.build_map_func()
            elif cmd == "cancel_mapping":
                self.cancel_mapping_func()
            elif cmd == "save_map":
                self.save_map_func()
            elif cmd == "change_map":
                self.change_map_func()
            elif cmd == "create_group":
                self.create_group_func()
            elif cmd == "rename_group":
                self.rename_group_func()
            elif cmd == "rename_map":
                self.rename_map_func()
            elif cmd == "delete_map":
                self.delete_map_func()
            elif cmd == "delete_group":
                self.delete_group_func()
            elif cmd == "clear_route":
                self.clear_route_func()
            elif cmd == "save_route":
                self.save_route_func()
            elif cmd == "delete_route":
                self.delete_route_func()
            elif cmd == "change_route":
                self.change_route_func()
            elif cmd == "rename_route":
                self.rename_route_func()
            else:
                # ignore waypoint handler commands (other node)
                if cmd in ("start", "stop", "follow_route", "next_point", "previous_point", "home"):
                    return
                self._ui_warn("UNKNOWN_CMD", "Unknown command received.", cmd=cmd)

        except Exception as e:
            self._ui_error("UI_CB_EXC", "Exception in ui_callback (caught).", err=str(e))


def main(args=None):
    rclpy.init(args=args)
    node = UIFoldersHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
