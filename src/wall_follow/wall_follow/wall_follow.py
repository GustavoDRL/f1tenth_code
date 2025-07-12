#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
import math
import time

class WallFollow(Node):

    def __init__(self):
        super().__init__('wall_follow')

        plt.ion()  # Enable interactive mode for real-time updates
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Error')
        self.ax.set_title('Error vs. Time')
        self.ax.grid(True)

        # QoS compatível com LiDAR (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile  # Usar QoS compatível
        )

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',  # Topic where Ackermann commands will be published
            10
        )

        # PID parameters
        self.kp = 14.0
        self.ki = 0.007
        self.kd = 0.09

        # Store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error_list = []
        self.time_list = []
        
        # LIDAR measurement angle
        self.angleA = 75
        self.angleB = 85 
        self.dist=0.7

        # Start time
        self.start_time = time.time()

    def scan_callback(self, msg):
        error = self.get_error(msg)
        self.error_list.append(error)

        velocity = self.calculate_velocity(error)
        self.pid_control(error, velocity)

        # Calculate elapsed time
        current_time = time.time() - self.start_time
        self.time_list.append(current_time)

        # Plot the error vs. time
        #self.plot_error_vs_time()

    def get_range(self, range_data, angle):
        if not range_data:
            return 0.0
        if math.isnan(range_data[angle]):
            return 0.0
        elif range_data[angle] == float('inf'):
            return 0.0
        else:
            return range_data[angle]

    def get_range_by_angle(self, scan: LaserScan, angle: float): 
        if angle < scan.angle_min or angle > scan.angle_max:
            return 0.0
        possible_desired_angle = angle - (angle % scan.angle_increment)
        desired_range_index = int((possible_desired_angle - scan.angle_min) // scan.angle_increment)
        if desired_range_index < 0 or desired_range_index >= len(scan.ranges):
            return 0.0
        return scan.ranges[desired_range_index]

    def get_error(self, msg):
        a = self.get_range_by_angle(msg, math.radians(self.angleA))
        b = self.get_range_by_angle(msg, math.radians(self.angleB))

        # Debug: verificar se as leituras são válidas
        if a == 0.0 or b == 0.0:
            print(f"DEBUG: Leituras inválidas - a: {a:.3f}, b: {b:.3f}")
            return 0.0

        teta = math.radians(self.angleB - self.angleA)
        sin_teta = math.sin(teta)
        
        # Proteção contra divisão por zero
        if abs(sin_teta) < 1e-6:
            print(f"DEBUG: sin(teta) muito pequeno: {sin_teta}")
            return 0.0

        alpha = math.atan((a * math.cos(teta) - b) / sin_teta)

        D = b * math.cos(alpha)
        error = self.dist - D
        
        # Debug opcional (descomente para ver valores)
        # print(f"a: {a:.3f}, b: {b:.3f}, alpha: {math.degrees(alpha):.1f}°, D: {D:.3f}, error: {error:.3f}")
        
        return error

    def calculate_velocity(self, error):
        if error < 0.5:
            velocity = 1.0 
        else:
            velocity = 0.5
        return velocity

    def pid_control(self, error, velocity):
        self.integral += error
        angle = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        self.prev_error = error

        # Publish Ackermann commands
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.drive.speed = velocity
        ackermann_cmd.drive.steering_angle = math.radians(-angle)
        print(f"Error: {error:.3f}, Angle: {angle:.3f}, Speed: {velocity:.3f}")
        self.publisher.publish(ackermann_cmd)

    def plot_error_vs_time(self):
        self.line.set_xdata(self.time_list)
        self.line.set_ydata(self.error_list)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
