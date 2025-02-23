#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
import csv, os
import time
import math
import yaml
import asyncio
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses

from nav2_msgs.action import FollowWaypoints

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance, Quaternion
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

PI = math.pi

class WayPointMover(Node):
    def __init__(self):
        super().__init__('way_points_handler')
        
        self.waypoints = []
        self.qos_profile = QoSProfile(
            depth=10, 
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  # Match subscriber
        )
        self.current_wp = 0
        self.on_the_route = False
        self.break_mission = False
        self.current_files = os.path.join(get_package_share_directory('ui_package'), 'param', 'current_map_route.yaml')

        self.charge_connected = False
        self.current_pos = PoseWithCovariance()
        self.home_pose = PoseWithCovariance()

        # Declare parameters
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("needCheckCharger", False)
        self.declare_parameter("charge_station_connected_topic", "charge_station_connected")

        # Get parameter values
        self.odom_topic = self.get_parameter("odom_topic").value
        self.needCheckCharger = self.get_parameter("needCheckCharger").value
        self.charge_station_connected_topic = self.get_parameter("charge_station_connected_topic").value

        # Topics subscribers
        self.create_subscription(String, "ui_operation", self.ui_operation_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if self.needCheckCharger:
            self.create_subscription(Bool, self.charge_station_connected_topic, self.charge_station_callback, 10)

        # Topics publishers
        self.ui_message_pub = self.create_publisher(String, "/ui_message", self.qos_profile)
        self.poseArray_publisher = self.create_publisher(PoseArray, "/WPs_topic", 10)

        # Action client for Nav2's NavigateToPose
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.current_goal_handle = None  # Track the current goal handle

        self.get_logger().info('Connecting to Nav2 navigate_to_pose action server...')
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Failed to connect to navigate_to_pose action server')
        else:
            self.get_logger().info('CONNECTED to navigate_to_pose action server')

        self.get_logger().info("------------ Way points handler started ------------")
        self.goal_completed = False
    def charge_station_callback(self, msg):
        self.charge_connected = msg.data
        self.ui_message_pub.publish(String(data=f"Robot connected to charge station: {self.charge_connected}"))
        self.get_logger().info(f"Robot connected to charge station: {self.charge_connected}")

        if self.charge_connected:
            self.home_pose.pose.position = self.current_pos.pose.position
            self.home_pose.pose.orientation = self.current_pos.pose.orientation

    def get_cur_files(self):
        with open(self.current_files, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
        return data

    def read_wp(self):
        route_file = self.get_cur_files()["route_file"]

        del self.waypoints[:]
        with open(route_file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            for line in reader:
                current_pose = PoseWithCovarianceStamped()
                current_pose.pose.pose.position.x = float(line[0])
                current_pose.pose.pose.position.y = float(line[1])
                current_pose.pose.pose.position.z = float(line[2])  # instead z coord use type of point
                current_pose.pose.pose.orientation.x = float(line[3])
                current_pose.pose.pose.orientation.y = float(line[4])
                current_pose.pose.pose.orientation.z = float(line[5])
                current_pose.pose.pose.orientation.w = float(line[6])
                current_pose.pose.covariance[0] = float(line[7])
                current_pose.pose.covariance[1] = float(line[8])
                current_pose.pose.covariance[2] = float(line[9])
                point_type = float(line[7])
                purpouse = float(line[10])

                self.waypoints.append((current_pose, point_type, purpouse))

        if not self.waypoints:
            self.ui_message_pub.publish(String(data="The waypoint queue is empty."))

    def odom_callback(self, msg):
        self.current_pos.pose = msg.pose.pose


    def follow_func(self):
        """Sends all waypoints to the robot at once using `/follow_waypoints`."""
        if self.needCheckCharger and not self.charge_connected:
            self.ui_message_pub.publish(String(data="⚠️ Robot can't start. Connect to charge station."))
            self.get_logger().warn("⚠️ Robot can't start. Connect to charge station.")
            return

        if self.on_the_route:
            self.ui_message_pub.publish(String(data="⚠️ The robot is already on the route"))
            self.get_logger().warn("⚠️ The robot is already on the route.")
            return

        self.break_mission = False
        self.on_the_route = True
        self.current_wp = 0

        self.read_wp()
        if len(self.waypoints) == 0:
            self.ui_message_pub.publish(String(data="⚠️ No waypoints found. Aborting mission."))
            self.get_logger().warn("⚠️ No waypoints found! Aborting mission.")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = []

        for index, (waypoint, _, _) in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position = waypoint.pose.pose.position
            pose.pose.orientation = waypoint.pose.pose.orientation
            goal.poses.append(pose)
            self.get_logger().info(f"📍 Added Waypoint {index + 1}: x={pose.pose.position.x}, y={pose.pose.position.y}")

        self.get_logger().info(f"🚀 Sending {len(goal.poses)} waypoints to `/follow_waypoints`...")
        self.ui_message_pub.publish(String(data=f"🚀 Navigating through {len(goal.poses)} waypoints..."))

        future = self.follow_waypoints_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles the goal response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.ui_message_pub.publish(String(data="❌ Goal was rejected!"))
            self.get_logger().warn("❌ Goal was rejected!")
            return

        self.get_logger().info("✅ Goal accepted, monitoring progress...")
        self.ui_message_pub.publish(String(data="✅ Goal accepted. Moving to waypoints..."))
        
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handles the final result when navigation is complete."""
        result = future.result()
        if result is not None:
            self.get_logger().info("✅ Navigation completed successfully!")
            self.ui_message_pub.publish(String(data="✅ Route completed successfully!"))
        else:
            self.get_logger().error("❌ Navigation failed!")
            self.ui_message_pub.publish(String(data="❌ Navigation failed. Check logs."))

        self.on_the_route = False
        self.current_goal_handle = None

    def stop_func(self):
        """Cancels the current navigation goal and stops the robot."""
        self.get_logger().info("🚨 Canceling current route")
        self.ui_message_pub.publish(String(data="🚨 Canceling current route..."))
        self.break_mission = True

        if self.current_goal_handle:
            try:
                if isinstance(self.current_goal_handle, rclpy.task.Future):
                    self.get_logger().info("⏳ Waiting for goal handle before canceling...")
                    self.ui_message_pub.publish(String(data="⏳ Waiting for goal handle before canceling..."))
                    self.current_goal_handle = self.current_goal_handle.result()

                if self.current_goal_handle:
                    self.get_logger().info("🔴 Sending goal cancellation request...")
                    self.ui_message_pub.publish(String(data="🔴 Sending goal cancellation request..."))
                    cancel_future = self.current_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self.cancel_done)
                else:
                    self.get_logger().error("❌ No valid goal handle to cancel.")
                    self.ui_message_pub.publish(String(data="❌ No valid goal handle to cancel."))
            except Exception as e:
                self.get_logger().error(f"⚠️ ERROR in stop_func: {e}")
                self.ui_message_pub.publish(String(data=f"⚠️ ERROR in stop_func: {e}"))

    def cancel_done(self, future):
        """Handles the cancel response."""
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info("✅ Goal successfully canceled.")
            self.ui_message_pub.publish(String(data="✅ Goal successfully canceled."))
        else:
            self.get_logger().warn("⚠️ Goal cancellation failed.")
            self.ui_message_pub.publish(String(data="⚠️ Goal cancellation failed."))

        self.on_the_route = False
        self.current_goal_handle = None


    def cancel_done(self, future):
        """Handles the cancel response."""
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info("✅ Goal successfully canceled.")
        else:
            self.get_logger().warn("⚠️ Goal cancellation failed.")

        self.on_the_route = False
        self.current_goal_handle = None

    async def home_func(self):
        self.get_logger().info("Following to home position")
        self.ui_message_pub.publish(String(data="Following to home position"))

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position = self.home_pose.pose.position
        new_orientation = self.chang_point_dir(self.home_pose.pose.orientation)
        goal.pose.pose.orientation.x = new_orientation[0]
        goal.pose.pose.orientation.y = new_orientation[1]
        goal.pose.pose.orientation.z = new_orientation[2]
        goal.pose.pose.orientation.w = new_orientation[3]

        # Send goal and wait for result
        self.current_goal_handle = await self.follow_waypoints_client.send_goal_async(goal)
        await self.current_goal_handle.get_result_async()
    def next_point_func(self):
        """Moves to the next waypoint in the list."""
        if not self.on_the_route:
            self.ui_message_pub.publish(String(data="⚠️ No active navigation. Start a route first."))
            self.get_logger().warn("⚠️ No active navigation. Start a route first.")
            return

        if self.current_wp >= len(self.waypoints):
            self.ui_message_pub.publish(String(data="⚠️ Already at the last waypoint."))
            self.get_logger().warn("⚠️ Already at the last waypoint.")
            return
        print()
        self.current_wp += 1
        self._navigate_to_waypoint(self.current_wp)

    def previous_point_func(self):
        """Moves to the previous waypoint in the list."""
        if not self.on_the_route:
            self.ui_message_pub.publish(String(data="⚠️ No active navigation. Start a route first."))
            self.get_logger().warn("⚠️ No active navigation. Start a route first.")
            return

        if self.current_wp <= 1:
            self.ui_message_pub.publish(String(data="⚠️ Already at the first waypoint."))
            self.get_logger().warn("⚠️ Already at the first waypoint.")
            return

        self.current_wp -= 1
        self._navigate_to_waypoint(self.current_wp)

    def _navigate_to_waypoint(self, wp_index):
        """Navigates to a specific waypoint given its index."""
        waypoint, _, _ = self.waypoints[wp_index - 1]  # Get the waypoint (0-based index)

        self.get_logger().info(f"🚀 Navigating to waypoint {wp_index}...")
        self.ui_message_pub.publish(String(data=f"🚀 Navigating to waypoint {wp_index}..."))

        # Create a FollowWaypoints goal (not NavigateToPose)
        goal = FollowWaypoints.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = waypoint.pose.pose.position
        pose.pose.orientation = waypoint.pose.pose.orientation
        goal.poses = [pose]  # FollowWaypoints expects a list of poses

        # Cancel current goal before sending a new one
        if self.current_goal_handle:
            self.get_logger().info("🔴 Canceling previous goal before proceeding...")
            self.stop_func()
            time.sleep(1)  # Ensure proper cancellation

        future = self.follow_waypoints_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.current_goal_handle = future  # Track new goal


    def ui_operation_callback(self, msg):
        self.get_logger().info(f"ui_operation_callback: {msg.data}")

        if msg.data == "follow_route" or msg.data == "start":
            self.follow_func()
            # asyncio.create_task(self.follow_func())
        elif msg.data == "next_point":
            self.next_point_func()
        elif msg.data == "previous_point":
            self.previous_point_func()
        # elif msg.data == "home":
        #     asyncio.create_task(self.home_func())
        elif msg.data == "stop":
            self.stop_func()
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
def main(args=None):
    rclpy.init(args=args)
    controller = WayPointMover()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()