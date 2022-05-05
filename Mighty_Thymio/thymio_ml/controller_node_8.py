import math
from urllib.robotparser import RobotFileParser
import rclpy
from rclpy.node import Node
import tf_transformations
from rclpy.clock import Duration
from rclpy.clock import ROSClock
from rclpy.time import Time
from rclpy.clock import ClockType

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys

class ControllerNode8(Node):
    def __init__(self):
        super().__init__('controller_node_8')
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        self.radius = 0.3
        self.call_counter = 0
        self.const = 1

        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2

    def update_angular(self):
        self.angle_direction = -self.angle_direction
        
    def update_callback(self):
        self.call_counter += 1
        
        radius = 0.3
        thymio_vel = 0.2
        T = (2*math.pi*radius)/thymio_vel

        angular_vel = (2*math.pi)/T

        if self.call_counter > 965:
            self.const = -self.const
            self.call_counter = 0

        cmd_vel = Twist() 
        cmd_vel.linear.x  = angular_vel * self.radius # [m/s]
        cmd_vel.angular.z = self.const * angular_vel # [rad/s]

        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode8()
    node.start()
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
