import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import math

class RobotControllerServer(Node):
    def __init__(self):
        super().__init__("robot_controller_server")
        self.robot_initial_x = 6.81  # TODO: get this dynamically
        self.robot_initial_y = 18.3  # TODO: get this dynamically
        self.get_logger().info("Robot Controller Server has started.")
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.velocity_timer_ = self.create_timer(0.5, self.timer_callback)
        self.timer_callback_called = False

        self.constant_vehicle_velocity_x = 0.2
        self.constant_vehicle_angular_z = 0.5
        self.current_vehicle_velocity_x = 0
        self.current_vehicle_orientation = 270

    def timer_callback(self):
        # FIXME: need to change this to account for different points, not just one
        if not self.timer_callback_called:
            self.timer_callback_called = True
            self.move_vehicle_bwt_two_nodes((self.robot_initial_x, self.robot_initial_y), (8, 20))

    def move_vehicle_bwt_two_nodes(self, node1, node2):
        x1, y1 = node1
        x2, y2 = node2

        dx = x1 - x2
        dy = y1 - y2

        angle = (math.degrees(math.atan2(dy, dx)) - self.current_vehicle_orientation + 360) % 360
        distance = math.sqrt(dx**2 + dy**2)
        angle_rad = math.radians(angle)

        self.rotate_vehicle(angle_rad, distance)

    # step 1
    def rotate_vehicle(self, rad, distance):
        time_to_rotate = abs(rad / self.constant_vehicle_angular_z)
        self.get_logger().info(f"angle to turn: {math.degrees(rad)}, time: {time_to_rotate}")

        msg = Twist()
        msg.angular.z = self.constant_vehicle_angular_z
        self.velocity_publisher_.publish(msg)

        # move on to next step -> advance robot
        self.create_timer(time_to_rotate, lambda: self.advance_robot(distance))

    # step 2
    def advance_robot(self, distance):
        time_to_advance = distance / self.constant_vehicle_velocity_x
        self.get_logger().info(f"distance to travel: {distance}, time: {time_to_advance}")

        msg = Twist()
        msg.linear.x = self.constant_vehicle_velocity_x
        self.velocity_publisher_.publish(msg)

        # move on to next step -> stop robot
        self.create_timer(time_to_advance, self.stop_robot)

    # step 3
    def stop_robot(self):
        self.velocity_publisher_.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                                angular=Vector3(x=0.0, y=0.0, z=0.0)))
        self.get_logger().info("Stopping the robot")
        self.velocity_timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()