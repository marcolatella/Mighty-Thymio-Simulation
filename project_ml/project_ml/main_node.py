from mimetypes import init
from re import T
import rclpy
from rclpy.node import Node
import tf_transformations
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image

from math import sqrt, sin, cos, atan2, pi
from enum import Enum
from rclpy.task import Future
import sys, os, cv2


class ThymioState(Enum):
    INIT = 0
    FOLLOWING_LINE = 1

HOMEPATH = os.path.expanduser("~")
DATASET_PATH = HOMEPATH+'/dataset'

class ControllerNode(Node):
    def __init__(self):
        super().__init__('main_node')
                
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.camera = self.create_subscription(Image, 'camera', self.img_callback, 10)

        self.current_state = ThymioState.INIT

        self.gl_sens = None
        self.gr_sens = None

        self.ground_l = self.create_subscription(Range, 'ground/left', self.ground_l_cb, 10)
        self.ground_r = self.create_subscription(Range, 'ground/right', self.ground_r_cb, 10)

         
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
        self.done_future = Future()
        
        return self.done_future
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)

    def img_callback(self, msg):
        pass


    def ground_l_cb(self, msg):
        self.gl_sens = msg.range
    
    def ground_r_cb(self, msg):
        self.gr_sens = msg.range

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

    def init_state(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 2.0
        cmd_vel.angular.z = 0.0
        return cmd_vel


    def update_init_state(self):
        if self.current_state == ThymioState.INIT:
            if self.gr_sens == 1.0 or self.gr_sens == 1.0:
                self.get_logger().info(f"Line detected!")
                self.current_state = ThymioState.FOLLOWING_LINE
                self.get_logger().info(f"Entered state {self.current_state}")


    def follow_line(self):
        cmd_vel = Twist()
        if not self.gl_sens:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -1.5
        elif not self.gr_sens:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 1.5
        else:
            cmd_vel.linear.x = 2.0
            cmd_vel.angular.z = 0.0

        return cmd_vel


    def update_callback(self):
        if self.current_state == ThymioState.INIT:
            self.update_init_state()
            cmd_vel = self.init_state()

        if self.current_state == ThymioState.FOLLOWING_LINE:
            cmd_vel = self.follow_line()

        
        #self.get_logger().info(f"Left: {self.gl_sens}, Right: {self.gr_sens}")

        # Publish the command
        self.vel_publisher.publish(cmd_vel)

    


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    done = node.start()

    rclpy.spin_until_future_complete(node, done)


if __name__ == '__main__':
    main()
