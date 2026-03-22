#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import cv2

class MapDownsampler(Node):
    def __init__(self):
        super().__init__('gui_map_downsampler')
        
        # --- Parameters ---
        # Now supports floats (e.g., 1.5, 2.0, 3.5)
        self.declare_parameter('scale', 1.5)
        self.scale = self.get_parameter('scale').value

        # How many times per second the GUI gets an update
        self.declare_parameter('update_rate_hz', 1.0)
        self.rate_hz = self.get_parameter('update_rate_hz').value

        # State variables for the "Mailbox" throttle pattern
        self.latest_map_msg = None
        self.new_map_available = False

        # Standard Map QoS
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.get_logger().info(f"Starting optimized Map Relay (Scale: {self.scale}x, Rate: {self.rate_hz}Hz)")

        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos)
        self.pub = self.create_publisher(OccupancyGrid, '/map_gui', qos)

        # The Timer dictates the output speed, completely independent of the incoming speed
        timer_period = 1.0 / self.rate_hz if self.rate_hz > 0 else 1.0
        self.timer = self.create_timer(timer_period, self.process_and_publish)

    def map_cb(self, msg: OccupancyGrid):
        # 1. Just drop the map in the mailbox. Very fast, no blocking.
        self.latest_map_msg = msg
        self.new_map_available = True

    def process_and_publish(self):
        # 2. Check if there is a new map to process
        if not self.new_map_available or self.latest_map_msg is None:
            return
            
        msg = self.latest_map_msg
        self.new_map_available = False # Reset the flag

        if self.scale <= 1.0:
            self.pub.publish(msg)
            return

        try:
            # Convert to 2D numpy array
            data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

            # --- FIX: The "White Walls" Issue ---
            # Create a mask of the walls (100) and fatten them slightly 
            # so they survive the shrinking process.
            walls = (data == 100).astype(np.uint8)
            kernel = np.ones((2, 2), np.uint8) # 2x2 expansion kernel
            fat_walls = cv2.dilate(walls, kernel, iterations=1)
            
            # Re-apply fat walls to the original map data
            data[fat_walls == 1] = 100

            # --- FIX: Float Scaling ---
            # Calculate new dimensions and use OpenCV to resize
            new_width = int(msg.info.width / self.scale)
            new_height = int(msg.info.height / self.scale)
            
            # INTER_NEAREST preserves the sharp -1, 0, 100 values without blurring them
            downsampled = cv2.resize(data, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

            # Build and send the new lightweight message
            new_msg = OccupancyGrid()
            new_msg.header = msg.header
            new_msg.info.resolution = msg.info.resolution * self.scale
            new_msg.info.width = new_width
            new_msg.info.height = new_height
            new_msg.info.origin = msg.info.origin
            new_msg.data = downsampled.flatten().tolist()

            self.pub.publish(new_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing map: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapDownsampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()