import rclpy
from rclpy.node import Node
import csv
import time
import math
import yaml

from std_srvs.srv import Empty as Emp

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance, Quaternion
from std_msgs.msg import Empty, String, Bool, Float32
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlan_Request
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

PI = math.pi

class WayPointMover(Node):
    def __init__(self):
        super().__init__('way_points_handler')
        self.waypoints = []
        self.current_wp = 0
        self.on_the_route = False
        self.break_mission = False
        self.current_files = self.get_parameter_or('current_map_route', 'ui_package/param/current_map_route.yaml')

        self.charge_connected = False
        self.current_pos = PoseWithCovariance()
        self.home_pose = PoseWithCovariance()

        # Parameters
        self.needCheckCharger = self.get_parameter_or("needCheckCharger", False)

        # Subscribers
        self.create_subscription(String, "ui_operation", self.ui_operation_callback, 10)
        self.create_subscription(Odometry, self.get_parameter_or("odom_topic", "odom"), self.odom_callback, 10)
        if self.needCheckCharger:
            self.create_subscription(Bool, self.get_parameter_or("charge_station_connected_topic", "charge_station_connected"), self.charge_station_callback, 10)

        # Publishers
        self.ui_message_pub = self.create_publisher(String, "/ui_message", 10)
        self.poseArray_publisher = self.create_publisher(PoseArray, "/WPs_topic", 10)

        # Actions
        self.move_base_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Connecting to navigate_to_pose...')
        self.move_base_client.wait_for_server()
        self.get_logger().info('Connected to navigate_to_pose')
        self.get_logger().info("------------ Way points handler started ------------")

    def charge_station_callback(self, data):
        self.charge_connected = data.data
        self.ui_message_pub.publish(String(data=f"Robot connected to charge station: {self.charge_connected}"))

        if self.charge_connected:
            self.home_pose.pose.position = self.current_pos.pose.position
            self.home_pose.pose.orientation = self.current_pos.pose.orientation

    def get_cur_files(self):
        with open(self.current_files, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def read_wp(self):
        route_file = self.get_cur_files()["route_file"]
        self.waypoints.clear()
        with open(route_file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            for line in reader:
                current_pose = PoseWithCovarianceStamped()
                current_pose.pose.pose.position.x = float(line[0])
                current_pose.pose.pose.position.y = float(line[1])
                current_pose.pose.pose.position.z = float(line[2])
                current_pose.pose.pose.orientation.x = float(line[3])
                current_pose.pose.pose.orientation.y = float(line[4])
                current_pose.pose.pose.orientation.z = float(line[5])
                current_pose.pose.pose.orientation.w = float(line[6])
                current_pose.pose.covariance = [float(c) for c in line[7:13]]
                point_type = float(line[7])
                purpouse = float(line[10])
                self.waypoints.append((current_pose, point_type, purpouse))

        if not self.waypoints:
            self.ui_message_pub.publish(String(data="The waypoint queue is empty."))

    def odom_callback(self, data):
        self.current_pos.pose = data.pose.pose

    def follow_func(self):
        if self.needCheckCharger and not self.charge_connected:
            self.ui_message_pub.publish(String(data="Robot can't start. Please connect to charge station"))
            return

        if self.on_the_route:
            self.ui_message_pub.publish(String(data="The robot on the route already"))
            return

        self.break_mission = False
        self.on_the_route = True
        self.current_wp = 0

        self.read_wp()

        for index, (waypoint, point_type, purpouse) in enumerate(self.waypoints):
            if self.break_mission:
                self.ui_message_pub.publish(String(data="break_mission"))
                break

            self.ui_message_pub.publish(String(data=f"Following to {index + 1} waypoint..."))

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose = waypoint.pose.pose

            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()

            self.current_wp = index + 1

            if not self.break_mission:
                self.ui_message_pub.publish(String(data=f"Doing some action #{purpouse} on point {index + 1}"))
            time.sleep(0.1)

        time.sleep(1)
        self.ui_message_pub.publish(String(data="Current route was successfully completed"))
        self.on_the_route = False

    def next_wp_func(self):
        self.ui_message_pub.publish(String(data="Following to next waypoint..."))
        self.read_wp()

        self.current_wp += 1

        if not self.waypoints:
            self.ui_message_pub.publish(String(data="The waypoint queue is empty."))
            return

        if self.current_wp > len(self.waypoints):
            self.current_wp = 1

        self.ui_message_pub.publish(String(data=f"Following to {self.current_wp} waypoint..."))
        point, pointType, purpouse = self.waypoints[self.current_wp - 1]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose = point.pose.pose

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        time.sleep(0.1)

        self.ui_message_pub.publish(String(data=f"Point #{self.current_wp} successfully reached"))
        self.ui_message_pub.publish(String(data=f"Doing some action #{purpouse} on point {self.current_wp}"))

    def ui_operation_callback(self, data):
        operation = data.data
        if operation == "follow_route" or operation == "start":
            self.follow_func()
        elif operation == "next_point":
            self.next_wp_func()
        elif operation == "stop":
            self.stop_func()

    def stop_func(self):
        self.ui_message_pub.publish(String(data="Canceling current route"))
        self.break_mission = True

        try:
            if self.move_base_client.get_result().status == GoalStatus.STATUS_EXECUTING:
                self.move_base_client.cancel_goal()
        except Exception as e:
            self.get_logger().error(f"Error in stop_func: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WayPointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
