#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import Twist, Vector3
from example_interfaces.msg import String, Float64
 
class RobotControllerServer(Node):
    def __init__(self):
        super().__init__("robot_controller_server")
        
        self.get_logger().info("Robot Controller Server has started.")
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publish_current_cmd_vel_callback()
 
    def publish_current_cmd_vel_callback(self):
        self.velocity_publisher_.publish(Twist(linear=Vector3(x=2.0, y=0.0, z=0.0),
                                                angular=Vector3(x=0.0, y=0.0, z=0.0)))
        
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerServer()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()