#!/usr/bin/env python3
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

from geometry_msgs.msg import PoseWithCovarianceStamped


class JoyToAckerman(Node):
    def __init__(self):
        super().__init__('joy_ackerman')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # Substitua 'joy_topic' pelo tópico correto onde os dados do joystick são publicados
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',  # Substitua 'ackermann_cmd' pelo tópico onde os comandos ackermann serão publicados
            10)
        self.max_speed = 7.0  # Limite de velocidade em m/s
        self.max_acceleration = 0
        self.max_angle = 0.32 # Limite de vangulo em rad
        self.max_angular_acceleration = 0.01
        self.controller_error = 0.1

        self.publisher_ = self.create_publisher(
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

        self.publisher_.publish(msg_map)
        self.get_logger().info('Initial pose published')
     
    
    def joy_callback(self, msg):
        # Eliminar valores muito próximos a zero para evitar movimentos indesejados
        if abs(msg.axes[1]) < self.controller_error:
            msg.axes[1] = 0.0
        if abs(msg.axes[3]) < self.controller_error:
            msg.axes[3] = 0.0
        #Return the car to initial position when PS button is pressed
        if msg.buttons[10] == 1 :
            self.publish_initial_pose()
        # Transformar dados do joystick em comandos de direção ackermann
        speed = self.max_speed * msg.axes[1]
        steering_angle = self.max_angle * msg.axes[3]

        # Publicar os comandos ackermann
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.drive.speed =  speed
        ackermann_cmd.drive.steering_angle  = steering_angle
        self.publisher.publish(ackermann_cmd)

def main(args=None):
    rclpy.init(args=args)
    joy_ackerman = JoyToAckerman()
    print("JoyToAckerman Initialized")
    rclpy.spin(joy_ackerman)
    joy_ackerman.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()