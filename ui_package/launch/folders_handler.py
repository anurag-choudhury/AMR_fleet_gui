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
from nav_msgs.srv import LoadMap

class UIFoldersHandler(Node):
    def __init__(self):
        super().__init__('ui_folders_handler')
        os.system("ros2 lifecycle set /map_server shutdown")

        self.WPs = []
        self.waypoints = []
        self.position = 0

        # Initialization of node
        self.get_logger().info("------------ UI folders handler started ------------")
        
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.ui_sub = self.create_subscription(String, "ui_operation", self.ui_callback, 10)
        self.new_wp_sub = self.create_subscription(PoseWithCovarianceStamped, "/new_way_point", self.new_way_point_callback, 10)
        self.nav_data_sub = self.create_subscription(Empty, "/nav_data_req", self.nav_data_callback, 10)
        self.wp_req_sub = self.create_subscription(Empty, "WP_req", self.WP_req_callback, 10)
        
        self.ui_pub = self.create_publisher(String, 'ui_message', 10)
        self.poseArray_pub = self.create_publisher(ArrayPoseStampedWithCovariance, "WayPoints_topic", 10)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav_data_pub = self.create_publisher(String, 'nav_data_resp', 10)

        self.maps_folder = os.path.join(self.get_package_share_directory('ui_package'), 'maps')
        self.routs_folder = os.path.join(self.get_package_share_directory('ui_package'), 'paths')
        self.current_files = os.path.join(self.get_package_share_directory('ui_package'), 'param', 'current_map_route.yaml')
        
        self.mappingCmd = self.get_parameter("mappingLaunch").value
        self.navigationCmd = self.get_parameter("navigationLaunch").value
        
        self.dict_cmd = None
        
        # TODO
        run_map_server_command = f"ros2 launch ui_package map_server.launch.py map_file:={self.get_cur_files()['map_file']}"  
        self.get_logger().info("--------------------------------------------")      
        self.get_logger().info(run_map_server_command)      
        self.get_logger().info("--------------------------------------------")      
        subprocess.Popen(run_map_server_command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        # --------

        self.get_logger().info('Connecting to map_server...')
        self.change_map_client = self.create_client(LoadMap, '/change_map')
        while not self.change_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /change_map service...')
        self.get_logger().info('CONNECTED to map_server')
        
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
            os.system("ros2 lifecycle set /amcl shutdown")
            os.system("ros2 lifecycle set /move_base shutdown")
            os.system("ros2 lifecycle set /map_server shutdown")
            time.sleep(1)
            
            subprocess.Popen(f"ros2 launch {self.mappingCmd}", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            time.sleep(3)
            self.ui_pub.publish(String(data="Move the robot along the perimeter of the room and in the "
                                "center using the control buttons and return robot to start position"))
        except Exception as e:
            self.get_logger().error(f"Error in build_map_func: {e}")

    def save_map_func(self):
        try:            
            self.ui_pub.publish(String(data="Saving map..."))
            map_path_to_save = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_folder_path_to_save = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"

            map_save_command = "ros2 run nav2_map_server map_saver_cli -f "
            cmd = f"{map_save_command}{map_path_to_save}"

            self.get_logger().info(cmd)                  
            os.system(cmd)
            
            os.mkdir(route_folder_path_to_save)
            self.set_cur_route("")
            self.WP_req_callback(Empty())
            self.ui_pub.publish(String(data="Map saved"))                

            # 1. Save new current map to file
            map_yaml_file = f"{map_path_to_save}.yaml"
            self.set_cur_map(map_path_to_save)
            self.get_logger().info(f"Current map was set to {map_yaml_file}")  

            # 2. Save current position
            current = self.position

            # 3. Reload
            os.system("ros2 lifecycle set /slam_gmapping shutdown")
            os.system("ros2 lifecycle set /map_server shutdown")

            self.ui_pub.publish(String(data="Localization..."))
            time.sleep(1)

            subprocess.Popen(f"ros2 launch {self.navigationCmd}", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            
            self.get_logger().info("Running saved map")  
            command = f"ros2 launch ui_package map_server.launch.py map_file:={map_yaml_file}"

            subprocess.Popen(command, stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)

            time.sleep(3)

            # 3. Set initial pose -> saved position
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = "map"
            current_pose.pose.pose = current  
            self.set_pose_pub.publish(current_pose)
            self.ui_pub.publish(String(data="You can set points or follow the route"))

            self.nav_data_pub.publish(String(data=self.get_paths()))

        except Exception as e:
            self.get_logger().error(f"Error in save_map_func: {e}")

    def change_map_func(self):
        try:                
            path_to_new_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            map_yaml_file = f"{path_to_new_map}.yaml"
            self.set_cur_map(path_to_new_map)

            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
            
            if len(routes_on_map) != 0:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
                self.set_cur_route(path_to_route)
            else:
                self.set_cur_route("")
                self.ui_pub.publish(String(data="No routes on the map"))
            self.WP_req_callback(Empty())

            self.get_logger().info(f"Changing map to {path_to_new_map}") 
            
            # Call the ROS2 service to change the map
            request = LoadMap.Request()
            request.map_url = map_yaml_file
            future = self.change_map_client.call_async(request)
            future.add_done_callback(self.change_map_callback)

        except Exception as e:
            self.get_logger().error(f"Error in change_map_func: {e}")

    def change_map_callback(self, future):
        try:
            response = future.result()
            if response.result:
                self.get_logger().info("Map changed successfully")
                self.nav_data_pub.publish(String(data=self.get_paths()))
            else:
                self.get_logger().error("Failed to change map")
        except Exception as e:
            self.get_logger().error(f"Error in change_map_callback: {e}")

    def create_group_func(self):
        try:
            os.mkdir(f"{self.routs_folder}/{self.dict_cmd['group']}")                
            os.mkdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            self.nav_data_pub.publish(String(data=self.get_paths()))
        except Exception as e:
            self.get_logger().error(f"Error in create_group_func: {e}")

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
            
            os.system("ros2 lifecycle set /map_server shutdown")  
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
            
            os.system("ros2 lifecycle set /map_server shutdown")  
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
        with open(self.current_files, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)  
        return data
def main(args=None):
    rclpy.init(args=args)
    controller = UIFoldersHandler()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

