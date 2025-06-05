#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import os
import shutil
import json
import time
import subprocess
import yaml
import csv

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from ui_package.msg import ArrayPoseStampedWithCovariance
from nav2_msgs.srv import LoadMap
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


import os
import subprocess
import time
import yaml
import shutil
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseWithCovariance
from std_msgs.msg import String, Empty
from nav_msgs.srv import LoadMap
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

class UIFoldersHandler(Node):
    def __init__(self):
        super().__init__('ui_folders_handler')
        
        # Initialize QoS profile
        self.qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  # Match subscriber
        )

        # Initialize waypoints and position
        self.WPs = []
        self.waypoints = []
        self.position = 0
        # os.system("killall map_server")
        # Service clients for lifecycle management
        self.map_server_get_state = self.create_client(GetState, '/map_server/get_state')
        self.map_server_change_state = self.create_client(ChangeState, '/map_server/change_state')
        self.change_map_client = self.create_client(LoadMap, '/map_server/load_map')
        print(self.change_map_client)
        # Ensure lifecycle services are available
        # Wait for services
        self.wait_for_service(self.map_server_get_state, "/map_server/get_state")
        self.wait_for_service(self.map_server_change_state, "/map_server/change_state")
        # self.wait_for_service(self.change_map_client, "/map_server/load_map")


        # Shutdown map_server if it's running
        # self.shutdown_lifecycle_node('/map_server')
        # Initialization of node
        self.get_logger().info("------------ UI folders handler started ------------")

        # Declare parameters with default values
        self.declare_parameter("mappingLaunch", "slam_toolbox online_async_launch.py")  # Default value
        self.declare_parameter("navigationLaunch", "ebot_nav2 ebot_bringup_launch.py")  # Default value
        self.declare_parameter("odom_topic", "/odom")  # Default value

        # Get parameter values
        self.mappingCmd = self.get_parameter("mappingLaunch").value
        self.navigationCmd = self.get_parameter("navigationLaunch").value
        self.odom_topic = self.get_parameter("odom_topic").value

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.ui_sub = self.create_subscription(String, "ui_operation", self.ui_callback, 10)
        self.new_wp_sub = self.create_subscription(PoseWithCovarianceStamped, "/new_way_point", self.new_way_point_callback, 10)
        self.nav_data_sub = self.create_subscription(Empty, "/nav_data_req", self.nav_data_callback, 10)
        self.wp_req_sub = self.create_subscription(Empty, "WP_req", self.WP_req_callback, 10)

        # Publishers
        self.ui_pub = self.create_publisher(String, 'ui_message', self.qos_profile)
        self.poseArray_pub = self.create_publisher(ArrayPoseStampedWithCovariance, "WayPoints_topic", 10)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav_data_pub = self.create_publisher(String, 'nav_data_resp', 10)

        # Initialize paths
        self.maps_folder = os.path.join(get_package_share_directory('ui_package'), 'maps')
        self.routs_folder = os.path.join(get_package_share_directory('ui_package'), 'paths')
        self.current_files = os.path.join(get_package_share_directory('ui_package'), 'param', 'current_map_route.yaml')
        
        # self.get_logger().info(f"Available services: {self.get_available_services()}")

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name} service...')

        # Initialize map server
        # self.change_map_client = self.create_client(LoadMap, '/map_server/load_map')
        # while not self.change_map_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for /map_server/load_map service...')
        # self.get_logger().info('CONNECTED to map_server')

        # # Start map server with the current map
        # self.start_map_server()

    def shutdown_lifecycle_node(self, node_name):
        """Shuts down a lifecycle node safely."""
        try:
            get_state_req = GetState.Request()
            future = self.map_server_get_state.call_async(get_state_req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() and future.result().current_state.id == 3:
                change_state_req = ChangeState.Request()
                change_state_req.transition.id = Transition.TRANSITION_DEACTIVATE
                self.map_server_change_state.call_async(change_state_req)

            change_state_req = ChangeState.Request()
            change_state_req.transition.id = Transition.TRANSITION_SHUTDOWN
            self.map_server_change_state.call_async(change_state_req)
        except Exception as e:
            self.get_logger().error(f"Error shutting down {node_name}: {e}")

    def start_map_server(self):
        """Start the map server with the current map."""
        try:
            current_map = self.get_cur_files()["map_file"]
            if current_map:
                run_map_server_command = f"ros2 launch nav2_map_server map_server.launch.py yaml_filename:={current_map}"
                self.get_logger().info(f"Starting map server: {run_map_server_command}")
                subprocess.Popen(run_map_server_command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            else:
                self.get_logger().warn("No map file specified. Map server not started.")
        except Exception as e:
            self.get_logger().error(f"Error starting map server: {e}")
        
    def odom_callback(self, data):
        self.position = data.pose.pose

    def nav_data_callback(self, data):
        time.sleep(0.5)
        self.nav_data_pub.publish(String(data=self.get_paths()))

    def new_way_point_callback(self, data):
        line =  f"{data.pose.pose.position.x},"
        line += f"{data.pose.pose.position.y},"
        line += f"{data.pose.pose.position.z},"
        line += f"{data.pose.pose.orientation.x},"
        line += f"{data.pose.pose.orientation.y},"
        line += f"{data.pose.pose.orientation.z},"
        line += f"{data.pose.pose.orientation.w},"
        line += f"{data.pose.covariance[0]},"
        line += f"{data.pose.covariance[1]},"
        line += f"{data.pose.covariance[2]}" 

        if(line not in self.WPs):
            self.WPs.append(line)

        if (len(self.WPs) == 1):
            self.ui_pub.publish(String(data="1 waypoint added to the route"))
        else:
            self.ui_pub.publish(String(data=str(len(self.WPs)) + " waypoints added to the route"))

    def convert_PoseArray(self, waypoints):
        poses = PoseArray()
        poses.header.frame_id = 'map'
        poses.poses = [pose.pose.pose for pose, purpose in waypoints]
        return poses

    def convert_PoseWithCovArray_to_PoseArrayCov(self, waypoints):
        poses = ArrayPoseStampedWithCovariance()
        for pose_arg, purpose in waypoints:       
            poses.poses.append(pose_arg)
        return poses

    def WP_req_callback(self, data):
        time.sleep(0.5)

        self.read_wp()  
        self.poseArray_pub.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
 
    def read_wp(self):
        route_file = self.get_cur_files()["route_file"]
        
        del self.waypoints[:]
        if len(route_file) != 0:
            with open(route_file, 'r') as file:
                reader = csv.reader(file, delimiter=',')
                for line in reader:
                    current_pose = PoseWithCovarianceStamped()
                    current_pose.header.frame_id = 'map'
                    current_pose.pose.pose.position.x = float(line[0])
                    current_pose.pose.pose.position.y = float(line[1])
                    current_pose.pose.pose.position.z = float(line[2])
                    current_pose.pose.pose.orientation.x = float(line[3])
                    current_pose.pose.pose.orientation.y = float(line[4])
                    current_pose.pose.pose.orientation.z = float(line[5])
                    current_pose.pose.pose.orientation.w = float(line[6])
                    current_pose.pose.covariance[0] = float(line[7])
                    current_pose.pose.covariance[1] = float(line[8])
                    current_pose.pose.covariance[2] = float(line[9])

                    self.waypoints.append((current_pose, float(line[10]))) 
        if self.waypoints == []:
            self.ui_pub.publish(String(data="The waypoint queue is empty."))
    
    def get_paths(self):
        files = {}
        for group in os.listdir(self.routs_folder):
            files[group] = []
            for i in os.listdir(os.path.join(self.routs_folder, group)):
                files[group].append({i: os.listdir(os.path.join(self.routs_folder, group, i))})
        
        data = self.get_cur_files()  
        if len(data["route_file"]) != 0:
            data = data["route_file"].split("/")[-3:]
            group, map, route = data[0], data[1], data[2].split(".")[0]
        elif len(data["map_file"]) != 0:
            data = data["map_file"].split("/")[-2:]
            group, map, route = data[0], data[1].split(".")[0], "Null"
        else:
            group, map, route = "Null", "Null", "Null"
        
        response = {"structure":[], "active_files":{"group":group,"map":map,"route":route}}

        for i, j in files.items():
            response["structure"].append({i: j})
        response = str(response).replace('\'', '\"')
        return response

    # ... (rest of the methods remain largely the same, with ROS1 API calls replaced by ROS2 equivalents)
    def build_map_func(self):
        try:
            self.poseArray_pub.publish(ArrayPoseStampedWithCovariance())
            self.ui_pub.publish(String(data="Mapping..."))
            
            # Shutdown AMCL, Nav2, and map_server
            # self.shutdown_lifecycle_node('/amcl')
            # self.shutdown_lifecycle_node('/bt_navigator')
            # self.shutdown_lifecycle_node('/map_server')
            
            # os.system("ros2 lifecycle set /map_server shutdown")
            os.system("killall map_server")
            # os.system("ros2 lifecycle set /amcl shutdown")
            os.system("killall async_slam_toolbox_node")
            time.sleep(1)
            
            # Start SLAM Toolbox for mapping
            subprocess.Popen(f"ros2 launch {self.mappingCmd}", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            time.sleep(3)
            
            self.ui_pub.publish(String(data="Move the robot along the perimeter of the room and in the center using the control buttons and return robot to start position"))
        except Exception as e:
            self.get_logger().error(f"Error in build_map_func: {e}")

    def save_map_func(self):
        try:
            self.ui_pub.publish(String(data="Saving map..."))
            map_path_to_save = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_folder_path_to_save = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"

            # Save the map using nav2_map_server
            map_save_command = "ros2 run nav2_map_server map_saver_cli -f "
            cmd = f"{map_save_command}{map_path_to_save}"
            self.get_logger().info(cmd)
            os.system(cmd)
            
            # Create route folder and reset current route
            os.mkdir(route_folder_path_to_save)
            self.set_cur_route("")
            self.WP_req_callback(Empty())
            self.ui_pub.publish(String(data="Map saved"))

            # Save new current map to file
            map_yaml_file = f"{map_path_to_save}.yaml"
            self.set_cur_map(map_path_to_save)
            self.get_logger().info(f"Current map was set to {map_yaml_file}")
            # Reset current route (optional, depending on your use case)
            self.set_cur_route("")  # Reset current route
            self.get_logger().info("Current route was reset")
            
            # Save current position
            current = self.position

            # Reload nodes
            # self.shutdown_lifecycle_node('/slam_toolbox')
            # self.shutdown_lifecycle_node('/map_server')
            os.system("killall async_slam_toolbox_node")
            time.sleep(1)

            # Start navigation stack
            # subprocess.Popen(f"ros2 launch {self.navigationCmd}", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            self.get_logger().info("Running saved map")
            
            # Start map server with the new map
            try:
                if not hasattr(self, "dict_cmd") or "group" not in self.dict_cmd or "map" not in self.dict_cmd:
                    self.get_logger().error("❌ Invalid map change request. 'group' or 'map' is missing.")
                    return


                # Call the service directly
                command = f"ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{{map_url: \"{map_yaml_file}\"}}'"
                self.get_logger().info(f"🚀 Executing: {command}")
                result = os.system(command)

                # Check the result of the system call
                if result == 0:
                    self.get_logger().info("✅ Map changed successfully!")
                else:
                    self.get_logger().error(f"❌ Failed to change map. Error code: {result}")

            except Exception as e:
                self.get_logger().error(f"🔥 Error in change_map_func: {e}")
            command = f"ros2 launch nav2_map_server map_server.launch.py yaml_filename:={map_yaml_file}"
            subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            time.sleep(3)

            # Set initial pose to saved position
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = "map"
            current_pose.pose.pose = current
            self.set_pose_pub.publish(current_pose)
            self.ui_pub.publish(String(data="You can set points or follow the route"))

            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in save_map_func: {e}")


    def get_available_services(self):
        services = self.get_service_names_and_types()
        return [service[0] for service in services]  # List only service names

    
    def get_change_map_client(self):
        if not hasattr(self, 'change_map_client') or self.change_map_client is None:
            self.change_map_client = self.create_client(LoadMap, "/map_server/load_map")
            self.get_logger().info("✅ Created new service client for /map_server/load_map")
        return self.change_map_client

    def change_map_func(self):
        """Directly calls the LoadMap service using os.system()."""
        try:
            if not hasattr(self, "dict_cmd") or "group" not in self.dict_cmd or "map" not in self.dict_cmd:
                self.get_logger().error("❌ Invalid map change request. 'group' or 'map' is missing.")
                return

            map_yaml_file = os.path.join(self.maps_folder, self.dict_cmd["group"], f"{self.dict_cmd['map']}.yaml")

            # Call the service directly
            command = f"ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{{map_url: \"{map_yaml_file}\"}}'"
            self.get_logger().info(f"🚀 Executing: {command}")
            result = os.system(command)

            # Check the result of the system call
            if result == 0:
                self.get_logger().info("✅ Map changed successfully!")
                # Update current map and route
                self.set_cur_map(os.path.join(self.maps_folder, self.dict_cmd["group"], self.dict_cmd["map"]))
                self.set_cur_route("")  # Reset current route since the map has changed
            else:
                self.get_logger().error(f"❌ Failed to change map. Error code: {result}")

        except Exception as e:
            self.get_logger().error(f"🔥 Error in change_map_func: {e}")

    def change_map_callback(self, future):
        """Callback function to handle the response of LoadMap service."""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Received empty response from LoadMap service.")
                return
            if response.result:
                self.get_logger().info("✅ Map changed successfully!")
            else:
                self.get_logger().error("❌ Failed to change map. Service returned False.")
        except Exception as e:
            self.get_logger().error(f"Exception in change_map_callback: {e}")



    def rename_map_func(self):
        try:
            old_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}" 

            old_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"  
            new_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}" 

            map_cur_path = None
            with open(f"{old_map_file}.yaml", 'r') as file:
                map_cur_path = yaml.load(file, Loader=yaml.FullLoader) 

            map_cur_path["image"] = f"{new_map_file}.pgm"

            with open(f"{old_map_file}.yaml", 'w') as file:
                yaml.dump(map_cur_path, file)

            os.rename(f"{old_map_file}.yaml", f"{new_map_file}.yaml")
            os.rename(f"{old_map_file}.pgm", f"{new_map_file}.pgm")

            os.rename(old_route_folder_file, new_route_folder_file)
            
            data = self.get_cur_files()
            if data["map_file"] == f"{old_map_file}.yaml":
                self.set_cur_map(new_map_file)
                self.set_cur_route(f"{new_route_folder_file}/{data['route_file'].split('/')[-1].split('.')[0]}")
            
            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in rename_map_func: {e}")

    def delete_map_func(self):
        try:
            map_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_map_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"   

            os.remove(f"{map_to_delete}.yaml")
            os.remove(f"{map_to_delete}.pgm")

            shutil.rmtree(route_map_folder_to_delete)
            
            # os.system("ros2 lifecycle set /map_server shutdown")  
            time.sleep(0.5)  
            maps_in_group = os.listdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            if len(maps_in_group) != 0:
                path_to_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}"
                self.set_cur_map(path_to_map)                
                self.get_logger().info(f"Current map: {path_to_map}")

                subprocess.Popen(f"ros2 launch ui_package map_server.launch.py map_file:={path_to_map}.yaml", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)

                routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}")   
                self.get_logger().info(f"routes_on_map: {routes_on_map}")
                
                if len(routes_on_map) != 0:
                    path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                    self.set_cur_route(path_to_route) 
                    self.get_logger().info(f"Current route: {path_to_route}")
                else:
                    self.set_cur_route("")
                    self.ui_pub.publish(String(data="No routes on the map"))

            else:
                self.set_cur_route("")
                self.set_cur_map("")
                self.ui_pub.publish(String(data="No maps in the group"))
            self.WP_req_callback(Empty())         
            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in delete_map_func: {e}")

    def delete_group_func(self):
        try:
            group_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}"
            route_group_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}"
            
            # os.system("ros2 lifecycle set /map_server shutdown")  
            time.sleep(0.5) 
            
            shutil.rmtree(group_to_delete)
            shutil.rmtree(route_group_folder_to_delete)

            groups_in_folder = os.listdir(f"{self.routs_folder}")   
            self.get_logger().info(f"groups_in_folder: {groups_in_folder}")

            if len(groups_in_folder) != 0:
                path_to_map = f"{self.maps_folder}/{groups_in_folder[0].split('.')[0]}"

                maps_in_group = os.listdir(path_to_map)
                if len(maps_in_group) != 0:
                    path_to_map = f"{path_to_map}/{maps_in_group[0].split('.')[0]}"
                    self.set_cur_map(path_to_map)                
                    self.get_logger().info(f"Current map: {path_to_map}")

                    subprocess.Popen(f"ros2 launch ui_package map_server.launch.py map_file:={path_to_map}.yaml", stdout=subprocess.PIPE,
                                shell=True, preexec_fn=os.setsid)

                    routes_on_map = os.listdir(f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}")   
                    self.get_logger().info(f"routes_on_map: {routes_on_map}")
                    
                    if len(routes_on_map) != 0:
                        path_to_route = f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                        self.set_cur_route(path_to_route) 
                        self.get_logger().info(f"Current route: {path_to_route}")
                    else:
                        self.set_cur_route("")
                        self.ui_pub.publish(String(data="No routes on the map"))

                else:
                    self.set_cur_map("")
                    self.ui_pub.publish(String(data="No maps in the group"))
            else:
                self.set_cur_map("")
                self.ui_pub.publish(String(data="No maps in the group"))
            self.WP_req_callback(Empty()) 

            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in delete_group_func: {e}")

    def clear_route_func(self):
        try:
            del self.WPs[:]
            self.ui_pub.publish(String(data="Waypoints cleared, please set new points on the map"))
        except Exception as e:
            self.get_logger().error(f"Error in clear_route_func: {e}")

    def save_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"

            with open(f"{path_to_route}.csv", 'w') as file:
                for WP in self.WPs:
                    pos = WP + ", 1"
                    file.write(pos + '\n')
            self.ui_pub.publish(String(data=f"{len(self.WPs)} waypoints saved"))
            self.get_logger().info(f"{len(self.WPs)} waypoints saved")

            self.set_cur_route(path_to_route)

            del self.WPs[:]
            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in save_route_func: {e}")

    def edit_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route'].split('.')[0]}"
            self.set_cur_route(path_to_route)                

            self.read_wp()           
            for point, purpose in self.waypoints:
                if purpose == 2: continue
                line =  f"{point.pose.pose.position.x},{point.pose.pose.position.y},{point.pose.pose.position.z},{point.pose.pose.orientation.x},{point.pose.pose.orientation.y},{point.pose.pose.orientation.z},{point.pose.pose.orientation.w},{point.pose.covariance[0]},{point.pose.covariance[1]},{point.pose.covariance[2]}"

                if line not in self.WPs:
                    self.WPs.append(line)
            
            self.poseArray_pub.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in edit_route_func: {e}")

    def delete_route_func(self):
        try:
            file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}.csv"
            
            os.remove(file) 
            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
            if len(routes_on_map) != 0:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
                self.set_cur_route(path_to_route)
            else:
                self.set_cur_route("")
                self.ui_pub.publish(String(data="No routes on the map"))

            self.WP_req_callback(Empty())
            self.nav_data_pub.publish(String(data=self.get_paths()))  
               
        except Exception as e:
            self.get_logger().error(f"Error in delete_route_func: {e}")

    def change_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}" 
            self.set_cur_route(path_to_route) 
            self.read_wp()       
            self.poseArray_pub.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints)) 
            self.nav_data_pub.publish(String(data=self.get_paths()))        
        except Exception as e:
            self.get_logger().error(f"Error in change_route_func: {e}")

    def rename_route_func(self):
        try:
            old_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_old']}.csv" 
            new_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_new']}.csv" 

            os.rename(old_file, new_file)
            self.set_cur_route(new_file.split(".")[0]) 
            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in rename_route_func: {e}")
    
    def ui_callback(self, data):
        self.get_logger().info("ui_callback")

        try:
            command = data.data.split("/") 
            self.get_logger().info(f"Command received: {command}")

            if len(command) > 1:  
                self.dict_cmd = json.loads(command[1])

            if command[0] == "build_map":
                self.get_logger().info("build_map")
                self.build_map_func()                
                                    
            elif command[0] == "save_map":  
                self.get_logger().info("save_map") 
                self.save_map_func()

            elif command[0] == "change_map":
                self.get_logger().info("change_map")
                self.change_map_func() 

            elif command[0] == "create_group":
                self.get_logger().info("create_group") 
                self.create_group_func()    

            elif command[0] == "rename_map":
                self.get_logger().info("rename_map") 
                self.rename_map_func() 

            elif command[0] == "delete_map":
                self.get_logger().info("delete_map")
                self.delete_map_func() 

            elif command[0] == "delete_group":
                self.get_logger().info("delete_group")
                self.delete_group_func()  

            elif command[0] == "clear_route": 
                self.get_logger().info("clear_route")   
                self.clear_route_func()  

            elif command[0] == "save_route":
                self.get_logger().info("save_route")
                self.save_route_func()

            elif command[0] == "edit_route":
                self.get_logger().info("edit_route")
                self.edit_route_func()

            elif command[0] == "delete_route":
                self.get_logger().info("delete_route")
                self.delete_route_func() 

            elif command[0] == "change_route":
                self.get_logger().info("change_route")
                self.change_route_func()

            elif command[0] == "rename_route":
                self.get_logger().info("rename_route")
                self.rename_route_func()

            elif command[0] == "rename_group":
                self.get_logger().info("rename_group")
                self.rename_group_func()

        except Exception as e:
            self.get_logger().error(f"Error in ui_callback: {e}")
    
    # -----------------------------------------------------------
    def set_cur_map(self, map_name):
        data = self.get_cur_files()  
        if len(map_name) == 0:
            data["map_file"] = ""
        else:
            data["map_file"] = f"{map_name}.yaml"
            
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)
     
    def set_cur_route(self, route_name):
        data = self.get_cur_files()
        if len(route_name) == 0:
            data["route_file"] = ""
        else:
            data["route_file"] = f"{route_name}.csv"        
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)  
  
    def get_cur_files(self):
        try:
            with open(self.current_files, 'r') as file:
                data = yaml.load(file, Loader=yaml.FullLoader)
        except Exception as e:
            self.get_logger().error(f"Error reading current_map_route.yaml: {e}")
            data = {"map_file": "", "route_file": ""}
        return data
def main(args=None):
    rclpy.init(args=args)
    controller = UIFoldersHandler()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

