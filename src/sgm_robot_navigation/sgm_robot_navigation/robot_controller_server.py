import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sgm_robot_interfaces.msg import MarkerNode, MapInformation

import math
import numpy as np

class RobotControllerServer(Node):
    def __init__(self):
        super().__init__("robot_controller_server")
        self.current_vehicle_x = 6.81  # TODO: get this dynamically
        self.current_vehicle_y = 18.3  # TODO: get this dynamically
        self.get_logger().info("Robot Controller Server has started.")
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.current_node_index = 0
        self.nodes = [(self.current_vehicle_x, self.current_vehicle_y), (8, 20), (9, 15)]

        self.timer_callback_called = False

        self.constant_vehicle_velocity_x = 0.2
        self.constant_vehicle_angular_z = 0.5
        self.current_vehicle_velocity_x = 0
        self.current_vehicle_orientation = 270
        
        self.rotate_timer_ = None
        self.advance_timer_ = None
        # self.velocity_timer_ = self.create_timer(0.5, self.timer_callback)
        
        self.marker_nodes_subscriber = self.create_subscription(
            MapInformation,
            'sgm_map',
            self.get_marker_nodes_information_callback,
            10
        )
        self.marker_nodes_information = []
        
    def get_marker_nodes_information_callback(self, msg):
        # grab information about the nodes when we the map and markers spawn
        # self.get_logger().info(f"info: {msg}")
        self.marker_nodes_information = msg.marker_nodes
        
        nearest_node_idx = self.find_nearest_node((self.current_vehicle_x,self.current_vehicle_y), self.marker_nodes_information)
        self.get_logger().info(f"nearest node is {nearest_node_idx}")

        self.move_vehicle_bwt_two_nodes((self.current_vehicle_x,self.current_vehicle_y), 
                                        (self.marker_nodes_information[nearest_node_idx].position_x, self.marker_nodes_information[nearest_node_idx].position_y))
    
    def find_nearest_node(self, point, sparse_nodes):
        def calculate_euclidean_distance(node1, node2):
            return np.linalg.norm(node1 - node2)
        
        min_distance = float('inf')
        nearest_node_idx = None
        for idx, node in enumerate(sparse_nodes):
            distance = calculate_euclidean_distance(np.array(point), np.array([node.position_x, node.position_y]))
            if distance < min_distance:
                min_distance = distance
                nearest_node_idx = node.index
        return nearest_node_idx
        
    def timer_callback(self):
        if not self.timer_callback_called and self.current_node_index + 1 < len(self.nodes):
            self.timer_callback_called = True
            self.move_vehicle_bwt_two_nodes(self.nodes[self.current_node_index], self.nodes[(self.current_node_index + 1)])
        elif self.current_node_index + 1 >= len(self.nodes):
            self.get_logger().info("end of loop")

    def move_vehicle_bwt_two_nodes(self, node1, node2):
        def rotate_vehicle(angle, time):
            self.get_logger().info(f"angle to turn: {angle}, time: {time}")
            self.velocity_publisher_.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                                     angular=Vector3(x=0.0, y=0.0, z=self.constant_vehicle_angular_z)))
            
            # update old info
            self.current_vehicle_orientation = (self.current_vehicle_orientation + angle) % 360
            
            if self.rotate_timer_ is not None:
                self.rotate_timer_.cancel()

        def advance_robot(distance, time, new_node):
            self.get_logger().info(f"distance to travel: {distance}, time: {time}")
            
            self.velocity_publisher_.publish(Twist(linear=Vector3(x=self.constant_vehicle_velocity_x, y=0.0, z=0.0),
                                                     angular=Vector3(x=0.0, y=0.0, z=0.0)))
            
            # update old info
            self.current_vehicle_x = new_node[0]
            self.current_vehicle_y = new_node[1]     
            
            if self.advance_timer_ is not None:
                self.advance_timer_.cancel()

        def stop_robot():
            self.velocity_publisher_.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                                    angular=Vector3(x=0.0, y=0.0, z=0.0)))
            self.get_logger().info(f"Stopping the robot. Current position: ({self.current_vehicle_x},{self.current_vehicle_y}; Orientation: {self.current_vehicle_orientation})")
            self.current_node_index = (self.current_node_index + 1) 
            
            self.timer_callback_called = False
        
        x1, y1 = node1
        x2, y2 = node2

        dx = x1 - x2
        dy = y1 - y2

        angle = (math.degrees(math.atan2(dy, dx)) - self.current_vehicle_orientation + 360) % 360
        distance = math.sqrt(dx**2 + dy**2)
        angle_rad = math.radians(angle)

        # rotation
        time_to_rotate = abs(angle_rad / self.constant_vehicle_angular_z)
        self.rotate_timer_ = self.create_timer(0.1, lambda: rotate_vehicle(angle, time_to_rotate))
        
        # advancement
        time_to_advance = distance / self.constant_vehicle_velocity_x
        self.advance_timer_ = self.create_timer(0.1 + time_to_rotate, lambda: advance_robot(distance, time_to_advance, node2))
        
        # stopping
        self.create_timer(0.1 + time_to_rotate + time_to_advance, stop_robot)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()