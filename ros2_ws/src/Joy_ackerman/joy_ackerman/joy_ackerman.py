#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import ompl.base as ob
import ompl.geometric as og
import math
from tf_transformations import euler_from_quaternion

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        
        # Set up logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().info('Initializing Car Controller Node')
        
        # Initialize subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', 
                                               self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom',
                                               self.odom_callback, 10)
        
        # Initialize publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Initialize sensor data storage
        self.latest_scan = None
        self.current_pose = None
        self.previous_steering = 0.0
        self.obstacle_points = []  # Store transformed obstacle points
        
        # Car parameters
        self.car_length = 0.58  # Length of the car in meters
        self.car_width = 0.31   # Width of the car in meters
        self.safety_margin = 0.2  # Additional safety margin around the car
        self.max_speed = 1.5     # Reduced max speed for better control
        self.min_speed = 0.5
        self.max_steering_angle = 0.4
        self.steering_smoothing = 0.3
        
        # Planning parameters
        self.planning_frequency = 10.0
        self.planning_time = 0.2
        self.goal_distance = 2.0  # Reduced for better local planning
        
        # Set up OMPL
        self.setup_ompl_planner()
        
        # Create planning timer
        self.create_timer(1.0/self.planning_frequency, self.planning_loop)
        self.get_logger().info('Car Controller Node initialized successfully')

    def setup_ompl_planner(self):
        try:
            self.space = ob.SE2StateSpace()
            
            # Set bounds for local planning
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(-5.0)
            bounds.setHigh(5.0)
            self.space.setBounds(bounds)
            
            self.ss = og.SimpleSetup(self.space)
            self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
            
            # Configure RRT
            planner = og.RRT(self.ss.getSpaceInformation())
            planner.setRange(1.0)  # Smaller range for finer motion
            planner.setGoalBias(0.1)  # Less goal bias for better obstacle avoidance
            self.ss.setPlanner(planner)
            
            self.get_logger().info('OMPL planner setup completed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup OMPL planner: {str(e)}')
            raise

    def update_obstacle_points(self):
        """Transform laser scan points to obstacle points in car's frame"""
        if self.latest_scan is None or self.current_pose is None:
            return

        self.obstacle_points = []
        
        # Get current car position and orientation
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        orientation_q = self.current_pose.orientation
        _, _, car_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Process laser scan points
        for i, range_val in enumerate(self.latest_scan.ranges):
            # Skip invalid measurements
            if range_val < self.latest_scan.range_min or range_val > self.latest_scan.range_max:
                continue

            # Calculate angle of this laser beam
            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
            
            # Calculate point position in car's frame
            point_x = range_val * math.cos(angle)
            point_y = range_val * math.sin(angle)
            
            self.obstacle_points.append((point_x, point_y))

    def is_state_valid(self, state):
        """Check if a state is collision-free"""
        if not self.obstacle_points:
            return False

        # Get the state values
        x = state.getX()
        y = state.getY()
        yaw = state.getYaw()

        # Define car corners (simplified rectangular shape)
        half_length = (self.car_length / 2.0) + self.safety_margin
        half_width = (self.car_width / 2.0) + self.safety_margin

        # Calculate car corners in local frame
        corners = [
            (-half_length, -half_width),
            (-half_length, half_width),
            (half_length, half_width),
            (half_length, -half_width)
        ]

        # Transform corners to state's frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        transformed_corners = []
        for cx, cy in corners:
            # Rotate
            rx = cx * cos_yaw - cy * sin_yaw
            ry = cx * sin_yaw + cy * cos_yaw
            # Translate
            rx += x
            ry += y
            transformed_corners.append((rx, ry))

        # Check for collision with obstacle points
        for obstacle_x, obstacle_y in self.obstacle_points:
            if self.point_in_rectangle(obstacle_x, obstacle_y, transformed_corners):
                self.get_logger().debug(f'Collision detected at state ({x:.2f}, {y:.2f})')
                return False

        return True

    @staticmethod
    def point_in_rectangle(x, y, corners):
        """Check if point (x,y) is inside rectangle defined by corners"""
        def sign(p1x, p1y, p2x, p2y, p3x, p3y):
            return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y)

        d1 = sign(x, y, corners[0][0], corners[0][1], corners[1][0], corners[1][1])
        d2 = sign(x, y, corners[1][0], corners[1][1], corners[2][0], corners[2][1])
        d3 = sign(x, y, corners[2][0], corners[2][1], corners[3][0], corners[3][1])
        d4 = sign(x, y, corners[3][0], corners[3][1], corners[0][0], corners[0][1])

        has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0) or (d4 < 0)
        has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0) or (d4 > 0)

        return not (has_neg and has_pos)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.update_obstacle_points()
        self.get_logger().debug(f'Updated obstacle points from scan with {len(msg.ranges)} points')

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.update_obstacle_points()
        self.get_logger().debug(
            f'Updated pose: x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}'
        )

    def get_goal_state(self):
        """Generate goal state considering current heading"""
        if self.current_pose is None:
            return None

        # Get current orientation
        orientation_q = self.current_pose.orientation
        _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, 
                                                 orientation_q.z, orientation_q.w])

        # Calculate goal in car's direction
        goal_x = self.current_pose.position.x + self.goal_distance * math.cos(current_yaw)
        goal_y = self.current_pose.position.y + self.goal_distance * math.sin(current_yaw)

        # Create goal state
        goal = ob.State(self.space)
        goal().setX(goal_x)
        goal().setY(goal_y)
        goal().setYaw(current_yaw)
        return goal

    def planning_loop(self):
        if self.current_pose is None or not self.obstacle_points:
            self.get_logger().warn('No pose or obstacle data available')
            return

        try:
            # Set start state
            start = ob.State(self.space)
            start().setX(self.current_pose.position.x)
            start().setY(self.current_pose.position.y)
            orientation_q = self.current_pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, 
                                             orientation_q.z, orientation_q.w])
            start().setYaw(yaw)

            # Get goal state
            goal = self.get_goal_state()
            if goal is None:
                return

            # Plan path
            self.ss.setStartAndGoalStates(start, goal)
            solved = self.ss.solve(self.planning_time)

            if solved:
                self.get_logger().info('Found solution path')
                path = self.ss.getSolutionPath()
                self.publish_drive_command(path)
            else:
                self.get_logger().warn('No solution found')
                self.publish_stop_command()

        except Exception as e:
            self.get_logger().error(f'Error in planning loop: {str(e)}')
            self.publish_stop_command()

    def publish_drive_command(self, path):
        try:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            if path.getStateCount() > 1:
                current_state = path.getState(0)
                next_state = path.getState(1)

                # Calculate steering angle
                dx = next_state.getX() - current_state.getX()
                dy = next_state.getY() - current_state.getY()
                desired_yaw = math.atan2(dy, dx)
                current_yaw = current_state.getYaw()

                # Smooth steering
                raw_steering = self.normalize_angle(desired_yaw - current_yaw)
                steering_angle = (self.steering_smoothing * raw_steering + 
                                (1 - self.steering_smoothing) * self.previous_steering)
                steering_angle = max(min(steering_angle, self.max_steering_angle), 
                                  -self.max_steering_angle)

                # Adjust speed based on steering and proximity to obstacles
                closest_obstacle = min((math.sqrt(x*x + y*y) for x, y in self.obstacle_points), 
                                    default=float('inf'))
                speed_factor = min(closest_obstacle / 2.0, 1.0)  # Slow down near obstacles
                speed_factor *= (1.0 - (abs(steering_angle) / self.max_steering_angle) ** 2)
                speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor

                msg.drive.steering_angle = steering_angle
                msg.drive.speed = speed

                self.previous_steering = steering_angle

                self.get_logger().debug(
                    f'Publishing drive command - speed: {speed:.2f}, '
                    f'steering: {steering_angle:.2f}'
                )

            self.drive_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing drive command: {str(e)}')
            self.publish_stop_command()

    def publish_stop_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = CarController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()