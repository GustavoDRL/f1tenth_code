#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class PerfilLidarNode(Node):

    def __init__(self):
        super().__init__('perfil_lidar')

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        plt.ion()  # Enable interactive mode for real-time updates
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Angle (radians)')
        self.ax.set_ylabel('Range (meters)')
        self.ax.set_title('Lidar Scan Profile')
        self.ax.grid(True)
        self.incremento = 180  # Adjust the increment based on your needs

    
    def scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, int(self.incremento))
        array = np.zeros(self.incremento)  # Adjust the size based on the length of angles
        aux = self.incremento -1
        for i in range(0, len(msg.ranges), int(len(msg.ranges)/self.incremento)):
            array[aux] = msg.ranges[i]
            aux -= 1
        self.line.set_xdata(angles)
        self.line.set_ydata(array)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        print('Angulo minimo: ' + str(math.degrees(msg.angle_min)))
        print('Angulo maximo: ' + str(math.degrees(msg.angle_max)))
        print('num_pontos: ' + str(len(msg.ranges)))



def main(args=None):
    rclpy.init(args=args)
    print("perfil_lidar Initialized")
    perfil_lidar = PerfilLidarNode()
    rclpy.spin(perfil_lidar)
    perfil_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
