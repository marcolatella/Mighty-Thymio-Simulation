from re import T
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

from math import sqrt, sin, cos, atan2, pi
from enum import Enum
from rclpy.task import Future

import random

class ThymioState(Enum):
    MOVING = 1
    ROTATING = 2


import sys

class ControllerNode(Node):
    def __init__(self, threshhold = 0.08):
        super().__init__('controller_node')
        
        self.odom_pose = None
        self.odom_velocity = None
        self.threshhold = threshhold
                
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.sens_tolerance = 0.0005

        self.current_state = ThymioState.MOVING
        self.rotation_tolerance = 0.05
        self.const = 1

        self.starting_pose = None
        self.odom_tolerance = 0.1

        self.prox_center = None
        self.prox_center_left = None
        self.prox_center_right = None
        self.prox_left = None
        self.prox_right = None

        self.prox_center_sub = self.create_subscription(Range, 'proximity/center', self.proxCenter_cb, 10)
        self.prox_center_left_sub = self.create_subscription(Range, 'proximity/center_left', self.proxCenterLeft_cb, 10)
        self.prox_center_right_sub = self.create_subscription(Range, 'proximity/center_right', self.proxCenterRight_cb, 10)
        self.prox_left_sub = self.create_subscription(Range, 'proximity/left', self.proxLeft_cb, 10)
        self.prox_right_sub = self.create_subscription(Range, 'proximity/right', self.proxRight_cb, 10)
        self.prox_rear_left_sub = self.create_subscription(Range, 'proximity/rear_left', self.prox_rear_l_cb, 10)
        self.prox_rear_right_sub = self.create_subscription(Range, 'proximity/rear_right', self.prox_rear_r_cb, 10)
    

    def prox_rear_l_cb(self, msg):
        self.prox_r_l = msg


    def prox_rear_r_cb(self, msg):
        self.prox_r_r = msg


    def proxCenter_cb(self, msg):
        self.prox_center = msg

    
    def proxCenterLeft_cb(self, msg):
        self.prox_center_left = msg


    def proxCenterRight_cb(self, msg):
        self.prox_center_right = msg

    
    def proxLeft_cb(self, msg):
        self.prox_left = msg


    def proxRight_cb(self, msg):
        self.prox_right = msg

         
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
        
        #self.get_logger().info(
        #    "odometry: receeeeived pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        #)


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
    

    def prox_condition(self, range):
        return range >= 0 and range < self.threshhold

    def init_sensors(self):
        if self.prox_center:
            center_r = self.prox_center.range
            center_l_r = self.prox_center_left.range
            center_r_r = self.prox_center_right.range
            left_r = self.prox_left.range
            right_r = self.prox_right.range
        
        else:
            center_r, center_l_r, center_r_r, left_r, right_r = 0.2, 0.2, 0.2, 0.2, 0.2

        if center_r == -1.0:
            center_r = 0.2 
        if center_l_r == -1.0:
            center_l_r = 0.2
        if center_r_r == -1.0:
            center_r_r = 0.2
        if left_r == -1.0:
            left_r = 0.2
        if right_r == -1.0:
            right_r = 0.2

        sens = {
            'center_r': center_r,
            'center_l_r': center_l_r,
            'center_r_r': center_r_r,
            'left_r': left_r,
            'right_r': right_r
        }
        
        return sens

    def sensors_not_triggered(self, sens):
        for key, val in sens.items():
            if val != 0.2:
                return False
        return True

    def sensors_triggered(self, sens):
        for key, val in sens.items():
            if self.prox_condition(val):
                return True
        return False

    def obstacle_detected(self):
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.0 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]
        self.current_state = ThymioState.ROTATING
        self.const = 1 if random.randint(-1000, 1000) >= 0 else -1
        self.get_logger().info(f"Obstacle Detected! Entered state: ROTATING")
        self.get_logger().info(f"Start rotating")
        return cmd_vel

    def moving(self):
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.2 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]
        return cmd_vel

    def rotate(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = self.const * 0.5
        return cmd_vel

    def update_callback(self):

        sens = self.init_sensors()
        
        if self.current_state == ThymioState.MOVING and self.sensors_triggered(sens):
            cmd_vel = self.obstacle_detected()
        elif self.current_state == ThymioState.MOVING:
            cmd_vel = self.moving()
        if self.current_state == ThymioState.ROTATING and self.sensors_not_triggered(sens):
            self.get_logger().info(f"Obstacle no more detected!")
            cmd_vel = self.moving()
            self.current_state = ThymioState.MOVING
            self.get_logger().info(f"Entered state: MOVING")
        elif self.current_state == ThymioState.ROTATING:
            cmd_vel = self.rotate()

        # Publish the command
        self.vel_publisher.publish(cmd_vel)
        

def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    done = node.start()
    
    # Keep processings events until someone manually shuts down the node
   # try:
   #     rclpy.spin(node)
   # except KeyboardInterrupt:
   #     pass
    
    # Ensure the Thymio is stopped before exiting
    #node.stop()
    rclpy.spin_until_future_complete(node, done)


if __name__ == '__main__':
    main()


#ci sara da definre una fine con questi
# self.done_future.set_result(True)
# self.destroy_timer(self.timer)