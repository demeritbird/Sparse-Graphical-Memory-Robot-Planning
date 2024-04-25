#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sgm_robot_interfaces.action import RobotNavigate

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

 
 
class MinimalRobotClient(Node):
    def __init__(self):
        super().__init__("minimal_robot_client")
        self.minimal_robot_client = ActionClient(self, RobotNavigate, "robot_navigate")
        
        
    def send_goal(self, target_node):
            # Wait for the Server
            self.minimal_robot_client.wait_for_server()
            
            # Create a Goal
            goal = RobotNavigate.Goal()
            goal.target_node = target_node
            
            #Sending the Goal
            self.get_logger().info("Sending the Goal from Action Client")
            self.minimal_robot_client. \
                send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)
            
    def goal_response_callback(self, future):
        # check if goal was accepted or not.
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async(). \
                add_done_callback(self.goal_result_callback) 
        else:
            self.get_logger().info("Goal was rejected...")	
                    
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result Node: {result.result_node}; Result Time; {result.time_taken}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalRobotClient()
    node.send_goal(2)
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()