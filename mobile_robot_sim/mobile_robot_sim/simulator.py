import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import math

class MobileRobotSimulator(Node):
    def __init__(self):
        super().__init__('mobile_robot_simulator')

        # Declare and get parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_phi', 0.0)
        self.declare_parameter('dt', 0.01)

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.phi = self.get_parameter('initial_phi').value
        self.dt = self.get_parameter('dt').value

        # Debugging print statements
        self.get_logger().info(f"dt: {self.dt}")
        
        # Path storage
        self.path = Path()

        # Publishers and subscribers
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.u1 = 0.0  # Linear velocity
        self.u2 = 0.0  # Angular velocity
        self.timer = self.create_timer(self.dt, self.update_pose)

    # Rest of the class remains the same...


    def cmd_callback(self, msg):
        # Update control inputs from velocity command
        self.u1 = msg.linear.x
        self.u2 = msg.angular.z

    def update_pose(self):
        # Unicycle motion model
        self.x += math.cos(self.phi) * self.u1 * self.dt
        self.y += math.sin(self.phi) * self.u1 * self.dt
        self.phi += self.u2 * self.dt

        # Normalize the orientation
        self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))

        # Publish the robot's current pose
        self.publish_pose()

        # Broadcast TF transform
        self.broadcast_transform()

        # Publish the robot's path
        self.publish_path()

    def publish_pose(self):
        # Create and publish a PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'  # Use 'map' as the frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(self.phi / 2)
        pose.pose.orientation.w = math.cos(self.phi / 2)
        self.pose_publisher.publish(pose)

    def broadcast_transform(self):
        # Create and broadcast a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.phi / 2)
        t.transform.rotation.w = math.cos(self.phi / 2)
        self.tf_broadcaster.sendTransform(t)

    def publish_path(self):
        # Append the current pose to the path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(self.phi / 2)
        pose.pose.orientation.w = math.cos(self.phi / 2)

        self.path.header.stamp = pose.header.stamp
        self.path.header.frame_id = 'map'
        self.path.poses.append(pose)

        # Publish the path
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = MobileRobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
