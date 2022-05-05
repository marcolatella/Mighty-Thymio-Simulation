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

        
    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        #self.get_logger().info('okay, we are inside the callback, lets try to check at sensors')
        #self.get_logger().info(f'center proximity sensor {self.prox_center}')
        if self.prox_center:
            A = self.prox_center.range
            B = self.prox_center_left.range
            C = self.prox_center_right.range
            D = self.prox_left.range
            E = self.prox_right.range
        
        else:
            A, B, C, D, E = 0.2, 0.2, 0.2, 0.2, 0.2

        if A == -1.0:
            A = 0.2 
        if B == -1.0:
            B = 0.2
        if C == -1.0:
            C = 0.2
        if D == -1.0:
            D = 0.2
        if E == -1.0:
            E = 0.2
        
        if self.current_state == ThymioState.MOVING and (self.prox_condition(A) or self.prox_condition(B) or self.prox_condition(C) or self.prox_condition(D) or self.prox_condition(E)):
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.0 # [m/s]
            cmd_vel.angular.z = 0.0 # [rad/s]
            self.current_state = ThymioState.ROTATING
            self.get_logger().info(f"Wall Detected! Entered state: ROTATING")

        elif self.current_state == ThymioState.MOVING:
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.2 # [m/s]
            cmd_vel.angular.z = 0.0 # [rad/s]

        if self.current_state == ThymioState.ROTATING and A == 0.2 and B == 0.2 and C == 0.2 and D == 0.2 and E == 0.2:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.0
            self.current_state == ThymioState.MOVING
        
        elif self.current_state == ThymioState.ROTATING:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.1

        # Publish the command
        self.vel_publisher.publish(cmd_vel)
        

    def euclidean_distance(self, goal_pose, current_pose):
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))


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