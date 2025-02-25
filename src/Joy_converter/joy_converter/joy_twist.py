#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_twist')
        
        self.subscription = self.create_subscription(
            Joy,
            '/joy', # Topic where joystick data is published
            self.joy_callback,
            10)
            
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel', # Topic where Twist commands will be published
            10)
            
        self.max_linear_speed = 3.0 # Maximum linear speed in m/s
        self.max_angular_speed = 1.5 # Maximum angular speed in rad/s
        self.controller_error = 0.1
        
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
            
        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        msg_map = PoseWithCovarianceStamped()
        
        # Fill header
        msg_map.header.stamp.sec = 0
        msg_map.header.stamp.nanosec = 0
        msg_map.header.frame_id = 'map'
        
        # Fill pose
        msg_map.pose.pose.position.x = 0.0
        msg_map.pose.pose.position.y = 0.0
        msg_map.pose.pose.position.z = 0.0
        msg_map.pose.pose.orientation.x = 0.0
        msg_map.pose.pose.orientation.y = 0.0
        msg_map.pose.pose.orientation.z = 0.0
        msg_map.pose.pose.orientation.w = 0.0
        
        # Fill covariance
        msg_map.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.00, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        
        self.pose_publisher.publish(msg_map)
        self.get_logger().info('Initial pose published')
        
    def joy_callback(self, msg):
        # Filter out values close to zero to avoid undesired movements
        linear_input = msg.axes[1]
        angular_input = msg.axes[3]
        
        if abs(linear_input) < self.controller_error:
            linear_input = 0.0
        if abs(angular_input) < self.controller_error:
            angular_input = 0.0
            
        # Return the robot to initial position when PS button is pressed
        if msg.buttons[10] == 1:
            self.publish_initial_pose()
            
        # Transform joystick data into Twist commands
        linear_velocity = self.max_linear_speed * linear_input
        angular_velocity = self.max_angular_speed * angular_input
        
        # Publish Twist commands
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_velocity  # Forward/backward motion
        twist_cmd.linear.y = 0.0              # No sideways motion
        twist_cmd.linear.z = 0.0              # No vertical motion
        twist_cmd.angular.x = 0.0             # No roll
        twist_cmd.angular.y = 0.0             # No pitch
        twist_cmd.angular.z = angular_velocity # Yaw (steering)
        
        self.publisher.publish(twist_cmd)
        
def main(args=None):
    rclpy.init(args=args)
    joy_twist = JoyToTwist()
    print("JoyToTwist Initialized")
    rclpy.spin(joy_twist)
    joy_twist.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()