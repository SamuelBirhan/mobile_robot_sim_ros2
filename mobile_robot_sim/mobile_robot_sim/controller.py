import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')

        # Declare and get parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', math.pi/5.8)
        self.declare_parameter('forward_time', 5.0)
        self.declare_parameter('turn_time', 3.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.forward_time = self.get_parameter('forward_time').value
        self.turn_time = self.get_parameter('turn_time').value

        # Debugging print statements
        self.get_logger().info(f"Initial parameter linear_speed: {self.linear_speed}")
        self.get_logger().info(f"Initial parameter angular_speed: {self.angular_speed}")
        self.get_logger().info(f"Initial parameter forward_time: {self.forward_time}")
        self.get_logger().info(f"Initial parameter turn_time: {self.turn_time}")

        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_command)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.phase = 0  # Phase of square movement: 0 = move, 1 = turn


    def send_command(self):
        msg = Twist()
        elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        if self.phase == 0:  # Move forward
            if elapsed < self.forward_time:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            else:
                self.phase = 1
                self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        elif self.phase == 1:  # Turn
            if elapsed < self.turn_time:
                msg.linear.x = 0.0
                msg.angular.z = self.angular_speed
            else:
                self.phase = 0
                self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.publisher.publish(msg)

        # self.get_logger().info(f"Parameter linear_speed: {self.linear_speed}")
        # self.get_logger().info(f"Parameter angular_speed: {self.angular_speed}")
    

def main(args=None):
    rclpy.init(args=args)
    node = MobileRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
