#!/usr/bin/env python3
import os
import csv
import json
import time
import math
import yaml
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray

from nav2_msgs.action import FollowWaypoints, NavigateToPose
from std_srvs.srv import Trigger
import requests

class WayPointMover(Node):
    """
    Storage layout (matches folders_handler data_dir):
      ~/.ros/amr_gui_data/
        param/current_map_route.yaml
        paths/<group>/<map>/<route>.csv
    """

    def __init__(self):
        super().__init__("ui_way_points_handler")

        # -------------------------
        # Params (match folder handler approach)
        # -------------------------
        self.declare_parameter("data_dir", os.path.expanduser("~/.ros/amr_gui_data"))
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("needCheckCharger", False)
        self.declare_parameter("charge_station_connected_topic", "charge_station_connected")

        self.data_dir = os.path.expanduser(self.get_parameter("data_dir").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.needCheckCharger = bool(self.get_parameter("needCheckCharger").value)
        self.charge_station_connected_topic = self.get_parameter("charge_station_connected_topic").value

        # Rod services
        self.rod_extend_cli = self.create_client(Trigger, "/rod/extend")
        self.rod_retract_cli = self.create_client(Trigger, "/rod/retract")
        self.rod_stop_cli = self.create_client(Trigger, "/rod/stop")

        # Ensure folders exist
        self.param_dir = os.path.join(self.data_dir, "param")
        self.paths_dir = os.path.join(self.data_dir, "paths")
        os.makedirs(self.param_dir, exist_ok=True)
        os.makedirs(self.paths_dir, exist_ok=True)

        # Current selection YAML (NO share folder)
        self.current_files_yaml = os.path.join(self.data_dir, "current_map_route.yaml")

        # QoS: transient local for UI messages (latched-like)
        self.qos_transient = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # -------------------------
        # State
        # -------------------------
        self.waypoints = []           # list of dicts: {"pose": PoseStamped, "wait": float}
        self.current_wp_idx = 0
        self.on_the_route = False
        self.break_mission = False
        self.charge_connected = False
        self.current_goal_handle = None

        # Rod state topic parsing
        self.rod_seq_active = None  # int 0/1
        self.rod_mode = "None"
        self.rod_state_raw = ""
        self._rod_lock = threading.Lock()

        # Home mission thread control
        self._home_thread = None
        self._home_running = False

        # -------------------------
        # ROS I/O
        # -------------------------
        self.create_subscription(String, "ui_operation", self.ui_operation_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        if self.needCheckCharger:
            self.create_subscription(Bool, self.charge_station_connected_topic, self.charge_station_callback, 10)

        self.create_subscription(String, "/rod/state", self.on_rod_state, 10)

        self.ui_message_pub = self.create_publisher(String, "/ui_message", self.qos_transient)
        self.pose_array_pub = self.create_publisher(PoseArray, "/WPs_topic", 10)

        # Action clients
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._log_storage()

        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self._ui("ERROR", "NAV2_SERVER_DOWN", "❌ follow_waypoints action server not available.", {})
            self.get_logger().error("follow_waypoints action server not available")
        else:
            self._ui("INFO", "CONNECTED", "✅ Connected: follow_waypoints action server.", {})
            self.get_logger().info("✅ Connected: follow_waypoints action server")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose action server not available (Home may not work).")
        else:
            self.get_logger().info("✅ Connected: navigate_to_pose action server")

        # Startup signal
        self._ui("INFO", "STARTED", "Waypoints handler started", {"data_dir": self.data_dir})

    # -------------------------
    # Helpers
    # -------------------------
    def _call_trigger(self, client, label):
        """Non-blocking fire-and-forget trigger, for UI button actions."""
        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(f"{label}: service not available")
            return

        req = Trigger.Request()
        fut = client.call_async(req)

        def _done(_fut):
            try:
                res = _fut.result()
                if res.success:
                    self.get_logger().info(f"{label}: OK - {res.message}")
                else:
                    self.get_logger().warn(f"{label}: FAIL - {res.message}")
            except Exception as e:
                self.get_logger().error(f"{label}: ERROR - {e}")

        fut.add_done_callback(_done)

    def _log_storage(self):
        self.get_logger().info(f"Storage root: {self.data_dir}")
        self.get_logger().info(f"Routes root: {self.paths_dir}")
        self.get_logger().info(f"Current YAML: {self.current_files_yaml}")

    def _ui(self, level: str, code: str, message: str, details: dict | None = None):
        """Publish JSON ui_message in the same format as folders_handler."""
        payload = {
            "level": level,
            "code": code,
            "message": message,
            "details": details or {},
            "ts": time.time(),
        }
        self.ui_message_pub.publish(String(data=json.dumps(payload)))

    def _safe_load_yaml(self, path: str) -> dict:
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                return {}
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML {path}: {e}")
            return {}

    def _safe_write_yaml(self, path: str, data: dict):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                yaml.safe_dump(data, f)
        except Exception as e:
            self.get_logger().error(f"Failed to write YAML {path}: {e}")

    def _normalize_route_name(self, route: str) -> str:
        if not route:
            return ""
        return route[:-4] if route.endswith(".csv") else route

    def _resolve_route_file(self, cfg: dict) -> str:
        """
        Supports:
          - cfg["route_file"] absolute or relative
          - cfg["group"], cfg["map"], cfg["route"] (route without/with .csv)
        """
        route_file = (cfg.get("route_file") or "").strip()
        group = (cfg.get("group") or "").strip()
        map_name = (cfg.get("map") or "").strip()
        route = (cfg.get("route") or "").strip()

        candidates = []

        # 1) If route_file is absolute
        if route_file and os.path.isabs(route_file):
            candidates.append(route_file)

        # 2) route_file relative to data_dir
        if route_file and not os.path.isabs(route_file):
            candidates.append(os.path.join(self.data_dir, route_file.lstrip("/")))

        # 3) Build from group/map/route -> ~/.ros/amr_gui_data/paths/<group>/<map>/<route>.csv
        if group and map_name and route:
            r = self._normalize_route_name(route)
            candidates.append(os.path.join(self.paths_dir, group, map_name, f"{r}.csv"))
            candidates.append(os.path.join(self.paths_dir, group, map_name, f"{r}-route.csv"))
            candidates.append(os.path.join(self.paths_dir, group, map_name, f"{map_name}-route.csv"))

        # Pick first existing
        for c in candidates:
            if c and os.path.isfile(c):
                return c

        # If none exist, return "best guess" so logs are useful
        if candidates:
            return candidates[0]
        return ""

    def _publish_wp_pose_array(self):
        arr = PoseArray()
        arr.header.frame_id = "map"
        arr.header.stamp = self.get_clock().now().to_msg()
        for item in self.waypoints:
            arr.poses.append(item["pose"].pose)
        self.pose_array_pub.publish(arr)

    # =========================================================
    # Rod state parsing (SEQ busy/idle)
    # =========================================================
    def on_rod_state(self, msg: String):
        # Example: "HOMING:0|HOMED:1|LEP:0|REP:-1|SEQ:0|MODE:None"
        with self._rod_lock:
            self.rod_state_raw = msg.data
            try:
                parts = {}
                for kv in msg.data.split("|"):
                    if ":" in kv:
                        k, v = kv.split(":", 1)
                        parts[k] = v
                self.rod_seq_active = int(parts.get("SEQ", "0"))
                self.rod_mode = parts.get("MODE", "None")
            except Exception:
                pass

    def _wait_future(self, fut, timeout_s: float) -> bool:
        """Wait for an async future to complete without spinning (callbacks happen in main executor)."""
        ev = threading.Event()

        def _done(_):
            ev.set()

        fut.add_done_callback(_done)
        return ev.wait(timeout=timeout_s)

    def wait_for_rod_state_msg(self, timeout_s: float = 2.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s and not self.break_mission:
            with self._rod_lock:
                if self.rod_seq_active is not None:
                    return True
            time.sleep(0.05)
        return False

    def wait_rod_busy(self, timeout_s: float = 5.0) -> bool:
        """Wait until SEQ becomes 1 (sequence actually started)."""
        if not self.wait_for_rod_state_msg(2.0):
            self.get_logger().warn("⚠️ No /rod/state received yet")

        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s and not self.break_mission:
            with self._rod_lock:
                if self.rod_seq_active == 1:
                    return True
            time.sleep(0.05)
        return False

    def wait_rod_idle(self, timeout_s: float = 180.0) -> bool:
        """Wait until SEQ becomes 0 (sequence finished)."""
        if not self.wait_for_rod_state_msg(2.0):
            self.get_logger().warn("⚠️ No /rod/state received yet")

        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s and not self.break_mission:
            with self._rod_lock:
                if self.rod_seq_active == 0:
                    return True
            time.sleep(0.05)

        with self._rod_lock:
            last = self.rod_state_raw
        self.get_logger().error(f"❌ wait_rod_idle timeout. Last /rod/state: {last}")
        return False

    def call_trigger_blocking(self, client, name: str, timeout_s: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"❌ {name}: service unavailable")
            return False

        req = Trigger.Request()
        fut = client.call_async(req)

        if not self._wait_future(fut, timeout_s=timeout_s):
            self.get_logger().error(f"❌ {name}: service call timeout")
            return False

        resp = fut.result()
        if not resp:
            self.get_logger().error(f"❌ {name}: no response")
            return False
        if not resp.success:
            self.get_logger().error(f"❌ {name} failed: {resp.message}")
            return False

        self.get_logger().info(f"✅ {name} OK: {resp.message}")
        return True

    def start_and_wait_rod(self, action_name: str, client, timeout_s: float = 180.0) -> bool:
        """
        1) Ensure idle before starting
        2) Call service
        3) Wait SEQ=1 (busy)
        4) Wait SEQ=0 (idle)
        """
        # Ensure idle
        with self._rod_lock:
            busy = (self.rod_seq_active == 1)

        if busy:
            self.get_logger().warn("⚠️ Rod busy; waiting for idle BEFORE starting...")
            if not self.wait_rod_idle(timeout_s=timeout_s):
                return False

        if not self.call_trigger_blocking(client, action_name, timeout_s=5.0):
            # retry once after waiting idle
            self.get_logger().warn(f"⚠️ {action_name} failed; waiting idle and retrying once...")
            if not self.wait_rod_idle(timeout_s=timeout_s):
                return False
            if not self.call_trigger_blocking(client, action_name, timeout_s=5.0):
                return False

        if not self.wait_rod_busy(timeout_s=5.0):
            with self._rod_lock:
                last = self.rod_state_raw
            self.get_logger().error(f"❌ {action_name}: never saw SEQ:1. Last /rod/state: {last}")
            return False

        return self.wait_rod_idle(timeout_s=timeout_s)

    # =========================================================
    # NavigateToPose (blocking wait using events)
    # =========================================================
    def navigate_to_pose_blocking(self, pose_stamped: PoseStamped, timeout_s: float = 150.0) -> bool:
        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().error("❌ navigate_to_pose server not ready")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped

        send_future = self.nav_to_pose_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_s=5.0):
            self.get_logger().error("❌ send_goal_async timeout")
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("❌ Navigation goal rejected")
            return False

        # keep handle so STOP can cancel
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_s=timeout_s):
            self.get_logger().error("❌ Navigation result timeout")
            return False

        res = result_future.result()
        status = getattr(res, "status", None)
        if int(status) != 4:  # SUCCEEDED
            self.get_logger().error(f"❌ Navigation failed (status={status})")
            return False

        return True

    # -------------------------
    # Callbacks
    # -------------------------
    def odom_callback(self, msg: Odometry):
        pass

    def charge_station_callback(self, msg: Bool):
        self.charge_connected = bool(msg.data)
        self._ui("INFO", "CHARGE_STATUS", f"Robot connected to charge station: {self.charge_connected}", {})

    # -------------------------
    # Waypoint loading
    # -------------------------
    def read_wp(self) -> bool:
        """Load route CSV into self.waypoints. Returns True if loaded."""
        self.waypoints.clear()
        cfg = self._safe_load_yaml(self.current_files_yaml)
        route_path = self._resolve_route_file(cfg)

        if not route_path:
            self._ui("WARN", "NO_ROUTE_SELECTED", f"No route selected. Please select a route.:{self.current_files_yaml}", {})
            return False

        if not os.path.isfile(route_path):
            self._ui(
                "ERROR",
                "ROUTE_FILE_MISSING",
                f"❌ Route file not found: {route_path} ✅ Fix: Put routes here: {self.paths_dir}/<group>/<map>/<route>.csv and re-select route from UI.",
                {"route_file": route_path, "routes_root": self.paths_dir},
            )
            cfg["route_file"] = ""
            cfg["route"] = ""
            self._safe_write_yaml(self.current_files_yaml, cfg)
            return False

        try:
            with open(route_path, "r") as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    if len(row) < 11:
                        continue

                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()

                    pose.pose.position.x = float(row[0])
                    pose.pose.position.y = float(row[1])
                    pose.pose.position.z = float(row[2])
                    pose.pose.orientation.x = float(row[3])
                    pose.pose.orientation.y = float(row[4])
                    pose.pose.orientation.z = float(row[5])
                    pose.pose.orientation.w = float(row[6])

                    wait_s = float(row[10])
                    self.waypoints.append({"pose": pose, "wait": wait_s})

        except Exception as e:
            self.get_logger().error(f"Failed to read route CSV: {e}")
            self._ui("ERROR", "ROUTE_READ_FAIL", "Failed to read route file. Check logs.", {"error": str(e)})
            return False

        if not self.waypoints:
            self._ui("WARN", "WP_EMPTY", "The waypoint queue is empty.", {"route_file": route_path})
            return False

        self._publish_wp_pose_array()
        self._ui("INFO", "WP_LOADED", f"Loaded {len(self.waypoints)} waypoints.", {"route_file": route_path})
        return True

    # -------------------------
    # Follow / Stop (FollowWaypoints)
    # -------------------------
    def follow_func(self):
        if self.needCheckCharger and not self.charge_connected:
            self._ui("WARN", "CHARGER_REQUIRED", "⚠️ Robot can't start. Connect to charge station.", {})
            return

        if self.on_the_route:
            self._ui("WARN", "ALREADY_RUNNING", "⚠️ The robot is already on the route.", {})
            return

        if not self.read_wp():
            self._ui("WARN", "WP_NOT_READY", "⚠️ No waypoints found. Aborting mission.", {})
            return

        self.break_mission = False
        self.on_the_route = True
        self.current_wp_idx = 0

        goal = FollowWaypoints.Goal()
        goal.poses = [item["pose"] for item in self.waypoints]

        self._ui("INFO", "FOLLOW_START", f"🚀 Navigating through {len(goal.poses)} waypoints...", {})
        future = self.follow_waypoints_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self._ui("ERROR", "GOAL_REJECTED", "❌ Goal was rejected!", {})
            self.on_the_route = False
            self.current_goal_handle = None
            return

        self.current_goal_handle = goal_handle
        self._ui("INFO", "GOAL_ACCEPTED", "✅ Goal accepted. Moving to waypoints...", {})

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            res = future.result()
            status = getattr(res, "status", None)

            # 0=UNKNOWN, 1=ACCEPTED, 2=EXECUTING, 3=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if status == 4:
                self._ui("INFO", "FOLLOW_DONE", "✅ Route completed successfully!", {})
            elif status == 5:
                self._ui("WARN", "FOLLOW_CANCELED", "⚠️ Route canceled.", {})
            else:
                self._ui("ERROR", "FOLLOW_FAILED", "❌ Navigation failed/aborted. Check logs.", {"status": status})
        except Exception as e:
            self._ui("ERROR", "FOLLOW_RESULT_EXC", "❌ Exception in follow result callback.", {"error": str(e)})

        self.on_the_route = False
        self.current_goal_handle = None

    def stop_func(self):
        # Stop any home thread / loops
        self.break_mission = True

        if not self.current_goal_handle:
            self._ui("WARN", "NO_ACTIVE_GOAL", "⚠️ No active goal to cancel.", {})
            self.on_the_route = False
            return

        self._ui("INFO", "CANCEL_REQ", "🚨 Canceling current goal...", {})
        try:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_cb)
        except Exception as e:
            self._ui("ERROR", "CANCEL_EXC", "⚠️ ERROR while canceling goal.", {"error": str(e)})
            self.on_the_route = False
            self.current_goal_handle = None

    def _cancel_done_cb(self, future):
        try:
            resp = future.result()
            if hasattr(resp, "goals_canceling") and len(resp.goals_canceling) > 0:
                self._ui("INFO", "CANCELED", "✅ Goal successfully canceled.", {})
            else:
                self._ui("WARN", "CANCEL_FAIL", "⚠️ Goal cancellation failed.", {})
        except Exception as e:
            self._ui("ERROR", "CANCEL_CB_EXC", "⚠️ Exception in cancel callback.", {"error": str(e)})

        self.on_the_route = False
        self.current_goal_handle = None

    # -------------------------
    # Optional single-step: next/prev (best-effort)
    # -------------------------
    def _navigate_single_index(self, idx_1based: int):
        if not self.read_wp():
            return

        idx = idx_1based - 1
        if idx < 0 or idx >= len(self.waypoints):
            self._ui("WARN", "INDEX_OOR", "⚠️ Waypoint index out of range.", {"idx_1based": idx_1based})
            return

        if self.current_goal_handle:
            self.stop_func()
            time.sleep(0.2)

        goal = FollowWaypoints.Goal()
        goal.poses = [self.waypoints[idx]["pose"]]
        self._ui("INFO", "GOTO_WP", f"🚀 Navigating to waypoint {idx_1based}...", {})
        future = self.follow_waypoints_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def next_point_func(self):
        if self.current_wp_idx + 1 >= len(self.waypoints):
            self._ui("WARN", "LAST_WP", "⚠️ Already at the last waypoint.", {})
            return
        self.current_wp_idx += 1
        self._navigate_single_index(self.current_wp_idx + 1)

    def previous_point_func(self):
        if self.current_wp_idx - 1 < 0:
            self._ui("WARN", "FIRST_WP", "⚠️ Already at the first waypoint.", {})
            return
        self.current_wp_idx -= 1
        self._navigate_single_index(self.current_wp_idx + 1)

    # -------------------------
    # Home (tailored): Go WP one-by-one + Rod extend/retract (blocking) in a background thread
    # -------------------------
    def home_func(self):
        if self._home_running:
            self._ui("WARN", "HOME_ALREADY_RUNNING", "⚠️ Home mission already running.", {})
            return

        if self.on_the_route:
            self._ui("WARN", "ROUTE_ALREADY_RUNNING", "⚠️ Route already running. Stop it first.", {})
            return

        if not self.read_wp():
            self._ui("WARN", "WP_NOT_READY", "⚠️ No waypoints loaded. Can't run Home mission.", {})
            return

        if not self.nav_to_pose_client.server_is_ready():
            self._ui("WARN", "HOME_UNAVAILABLE", "navigate_to_pose server not ready.", {})
            return

        self.break_mission = False
        self._home_running = True

        self._ui(
            "INFO",
            "HOME_START",
            f"🏁 Home mission: Iterative WP loop + Rod Extend/Retract for {len(self.waypoints)} WPs (until stopped)",
            {},
        )

        def _runner():
            try:
                if not self.wait_for_rod_state_msg(timeout_s=3.0):
                    self.get_logger().warn("⚠️ No /rod/state received yet (continuing anyway)")

                # 🌍 POST MAP ONCE (at mission start)
                try:
                    self.post_waypoints_to_server()
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Map POST skipped due to error: {e}")

                lap = 0

                # 🔁 Repeat forever until break_mission is set
                while rclpy.ok() and not self.break_mission:
                    lap += 1
                    self._ui("INFO", "HOME_LAP_START", f"🔁 Starting lap {lap}", {"lap": lap})

                    for i, item in enumerate(self.waypoints, start=1):
                        if self.break_mission:
                            self._ui("WARN", "HOME_ABORTED", "🛑 Home mission stopped by user.", {"at_wp": i, "lap": lap})
                            return

                        ps = item["pose"]
                        self._ui("INFO", "HOME_GOTO", f"🚗 Going to WP {i}/{len(self.waypoints)} (lap {lap})", {"wp": i, "lap": lap})

                        ok = self.navigate_to_pose_blocking(ps, timeout_s=200.0)
                        if not ok:
                            self._ui("ERROR", "HOME_NAV_FAIL", f"❌ Navigation failed at WP {i} (lap {lap}). Skipping.", {"wp": i, "lap": lap})
                            continue

                        # 🌍 GET current location after reaching WP
                        try:
                            self.send_current_location(i)  # i is 1-based
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ CurrentLocation GET skipped: {e}")

                        try:
                            self._ui("INFO", "ROD_EXTEND", f"⬆️ Rod extend at WP {i} (lap {lap})", {"wp": i, "lap": lap})
                            if not self.start_and_wait_rod("extend", self.rod_extend_cli, timeout_s=180.0):
                                self._ui("ERROR", "ROD_EXTEND_FAIL", f"❌ Extend failed at WP {i} (lap {lap}). Attempting retract.", {"wp": i, "lap": lap})
                        finally:
                            self._ui("INFO", "ROD_RETRACT", f"⬇️ Rod retract at WP {i} (lap {lap})", {"wp": i, "lap": lap})
                            self.start_and_wait_rod("retract", self.rod_retract_cli, timeout_s=180.0)

                    self._ui("INFO", "HOME_LAP_DONE", f"✅ Lap {lap} completed.", {"lap": lap})

                # if loop exits normally
                self._ui("INFO", "HOME_STOPPED", "🛑 Home mission stopped.", {"lap": lap})

            finally:
                self._home_running = False
                self.current_goal_handle = None

        self._home_thread = threading.Thread(target=_runner, daemon=True)
        self._home_thread.start()

    
    def post_waypoints_to_server(self):
        url = "https://amr.wfp-smartwarehouse.in/api/api/map"
        payload = []

        for idx, item in enumerate(self.waypoints, start=1):
            ps = item["pose"]
            payload.append({
                "id": idx,
                "name": f"Location {idx}",
                "coordinates": {
                    "x": float(ps.pose.position.x),
                    "y": float(ps.pose.position.y),
                }
            })

        # close the loop like sample payload
        if payload:
            payload.append(payload[0])

        try:
            r = requests.post(url, json=payload, timeout=5)
            self.get_logger().info(f"📡 Map POST → {r.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ Map POST failed: {e}")


    def send_current_location(self, wp_index_1based: int):
        url = f"https://amr.wfp-smartwarehouse.in/api/currentLocation?location={wp_index_1based}"
        try:
            r = requests.get(url, timeout=3)
            self.get_logger().info(f"📍 CurrentLocation={wp_index_1based} → {r.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ Location GET failed: {e}")

    # -------------------------
    # UI Operation commands
    # -------------------------
    def ui_operation_callback(self, msg: String):
        cmd = (msg.data or "").strip()

        if cmd in ("start", "follow_route"):
            self.follow_func()
            return

        if cmd == "stop":
            self.stop_func()
            self._call_trigger(self.rod_stop_cli, "ROD_stop")
            return

        if cmd == "next_point":
            self.next_point_func()
            return

        if cmd == "previous_point":
            self.previous_point_func()
            return

        if cmd == "home":
            self.home_func()
            return

        if cmd == "rod_extend":
            self._call_trigger(self.rod_extend_cli, "ROD_EXTEND")
            return

        if cmd == "rod_retract":
            self._call_trigger(self.rod_retract_cli, "ROD_RETRACT")
            return

        # Ignore unknown commands silently


def main(args=None):
    rclpy.init(args=args)
    node = WayPointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
