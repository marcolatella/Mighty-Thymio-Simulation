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
    STOP = 1
    MOVING = 2
    PERPENDICULAR = 3
    FORWARD = 4


import sys

class ControllerNode(Node):
    def __init__(self, threshhold = 0.08):
        super().__init__('controller_node')
        
        self.odom_pose = None
        self.odom_velocity = None
        self.threshhold = threshhold
                
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.stopped = False
        self.is_perp = False
        self.sens_tolerance = 0.0005

        self.current_state = ThymioState.MOVING
        self.goal_yaw = None
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

    def wall_detected(self):
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.0 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]
        self.stopped = True
        self.current_state = ThymioState.STOP
        self.get_logger().info(f"Wall Detected! Entered state: STOP")
        return cmd_vel


    def moving_forward(self):
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.2 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]
        return cmd_vel

    def set_perpendicular(self, center_left_r, center_right_r):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cond = abs(center_left_r - center_right_r) <= self.sens_tolerance
        val = abs(center_left_r - center_right_r)
        if cond:
            cmd_vel.angular.z = 0.0
            self.current_state = ThymioState.PERPENDICULAR
            self.get_logger().info(f"Entered state: PERPENDICULAR")
            self.get_logger().info("Start rotating...")
        elif center_left_r > center_right_r:
            cmd_vel.angular.z = -val * 6
        else:
            cmd_vel.angular.z = val * 6
        return cmd_vel


    def set_perpendicular_inv(self):
        cmd_vel = Twist()
        cmd_vel.linear.x  = 0.0

        rr_range = self.prox_r_r.range
        rl_range = self.prox_r_l.range

        if rr_range == -1:
            rr_range = 0.2
        if rl_range == -1:
            rl_range = 0.3

        sens_diff = abs(rr_range - rl_range)
        if sens_diff <= self.sens_tolerance:
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x  = 0.2 # [m/s] 
            self.current_state = ThymioState.FORWARD
            self.starting_pose = self.pose3d_to_2d(self.odom_pose)
            self.get_logger().info("Rotation ended")
            self.get_logger().info(f"Entered state: FORWARD")
        elif rr_range > rl_range:
            cmd_vel.angular.z = -sens_diff * 6
        else:
            cmd_vel.angular.z = sens_diff * 6   

        return cmd_vel


    def step_away(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.0
        cmd_vel.linear.x  = 0.2 # [m/s] 
        pose2d = self.pose3d_to_2d(self.odom_pose)
        if self.euclidean_distance(pose2d, self.starting_pose) >= 2.0:    
            cmd_vel.linear.x  = 0.0 # [m/s]
            # cambiamento di stato come detto da Marco
            # chiusura del codice
            self.done_future.set_result(True)
            self.destroy_timer(self.timer) 
        return cmd_vel

    
    def euclidean_distance(self, goal_pose, current_pose):
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))


    def init_sensors(self):
        if self.prox_center:
            center_left_r = self.prox_center_left.range
            center_right_r = self.prox_center_right.range
        else:
            center_left_r, center_right_r = 0.2, 0.2
        if center_left_r == -1.0:
            center_left_r = 0.2
        if center_right_r == -1.0:
            center_right_r = 0.2

        return center_left_r, center_right_r
 

    def update_callback(self):

        center_left_r, center_right_r = self.init_sensors()

        if self.current_state == ThymioState.MOVING and (self.prox_condition(center_left_r) or self.prox_condition(center_right_r)):
            cmd_vel = self.wall_detected()
        elif self.current_state == ThymioState.MOVING:
            cmd_vel = self.moving_forward()
        if self.current_state == ThymioState.STOP:
            cmd_vel = self.set_perpendicular(center_left_r, center_right_r)
        if self.current_state == ThymioState.PERPENDICULAR:
            cmd_vel = self.set_perpendicular_inv()
        if self.current_state == ThymioState.FORWARD:
            cmd_vel = self.step_away()

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