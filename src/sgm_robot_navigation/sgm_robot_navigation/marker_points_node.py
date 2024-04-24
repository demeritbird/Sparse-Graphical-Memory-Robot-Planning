#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np
import cv2

class MarkerPointsNode(Node):
    def __init__(self):
        super().__init__('marker_points_node')
        
        self.get_logger().info("Marker Points Node has started.")
        
        self.map_height = 0
        self.map_width = 0
        self.map_data = [] # int8[]
        
        self.marker_publisher_ = self.create_publisher(Marker, 'visualization_marker', 5)
        self.timer_ = self.create_timer(1.0, self.publish_markers) 
        
        self.map_qos_profile_ = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_subscription_ = self.create_subscription(
            OccupancyGrid,
            'map',
            self.get_map_info_callback,
            qos_profile=self.map_qos_profile_
        )
        
    def get_map_info_callback(self, msg):    
        # map dimensions
        self.map_height = msg.info.height * msg.info.resolution
        self.map_width = msg.info.width * msg.info.resolution

        self.get_logger().info(f"Map Width: {self.map_height}, Map Height: {self.map_width}")
        
        # map wall information (walls -> 100, empty -> 0)
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        ## NOTE: for testing purposes, save image
        map_image = (self.map_data * 255 / 100).astype(np.uint8)  # Scale occupancy values to 0-255
        cv2.imwrite('output/occupancy_grid_map.png', map_image)

    def publish_markers(self):
        marker_msgs = []
        NO_MARKERS = 5  

        for i in range(NO_MARKERS):
            marker_msg = Marker()
            marker_msg.header.frame_id = 'map'
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.ns = 'markers'
            marker_msg.id = i 
            marker_msg.type = Marker.CYLINDER
            marker_msg.action = Marker.ADD
            
            # pose
            marker_msg.pose.position.x = 1.0 + i 
            marker_msg.pose.position.y = 2.0 + i
            marker_msg.pose.position.z = 0.5 
            
            # orientation
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            
            # scale
            marker_msg.scale.x = 1.0 
            marker_msg.scale.y = 1.0
            marker_msg.scale.z = 1.0 
            
            #color
            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            
            #lifetime
            marker_msg.lifetime.sec = 10

            marker_msgs.append(marker_msg)

        for marker_msg in marker_msgs:
            self.marker_publisher_.publish(marker_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPointsNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()