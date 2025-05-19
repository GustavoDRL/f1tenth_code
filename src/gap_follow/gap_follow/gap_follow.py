#!/usr/bin/env python3
"""ROS2 node for autonomous navigation using laser scan gap detection and PID control."""

import math
import time
from dataclasses import dataclass, field
from typing import List, Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy
from scipy.ndimage import median_filter
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

@dataclass(frozen=True)
class PIDParameters:
    """PID controller configuration parameters."""
    kp: float
    ki: float
    kd: float
    min_output: float
    max_output: float
    antiwindup_threshold: float = 0.8
    d_filter_alpha: float = 0.2

@dataclass
class PIDDebugData:
    """PID controller debug information container."""
    timestamp: float
    error: float
    p_term: float
    i_term: float
    d_term: float
    output: float
    compute_time: float

@dataclass
class ScanData:
    """Container for processed scan data."""
    timestamp: float
    target_angle: float
    ranges: np.ndarray
    is_valid: bool = True

class PIDController:
    """Robust PID controller implementation with anti-windup and safety features."""

    def __init__(self, params: PIDParameters, logger=None):
        self.params = params
        self.logger = logger
        self.reset()

    def reset(self):
        """Initialize controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.debug_data = []
        self.max_debug_samples = 1000
        self.d_filter = 0.0 

    def compute(self, error: float, current_time: Time) -> float:
        """Compute PID output with time-based differentiation."""
        try:
            if self.last_time is None:
                self.last_time = current_time
                self.prev_error = error
                return 0.0

            dt = 0.01  # 100Hz fixed rate
            self.last_time = current_time

            p_term = self.params.kp * error
            self.integral += error * dt
            raw_d_term = (error - self.prev_error) / dt

            self.d_filter = (1 - self.params.d_filter_alpha) * self.d_filter + \
                           self.params.d_filter_alpha * raw_d_term
            d_term = self.params.kd * self.d_filter

            output = p_term + (self.params.ki * self.integral) + d_term
            limited_output = self._clamp_output(output)

            self._store_debug_data(
                error=error,
                p_term=p_term,
                i_term=self.params.ki * self.integral,
                d_term=d_term,
                output=limited_output
            )
            
            self.prev_error = error
            return limited_output

        except Exception as e:
            print(f"PID computation error: {str(e)}")
            return 0.0

    def _clamp_output(self, output: float) -> float:
        """Apply output limits with anti-windup compensation."""
        clipped = np.clip(output, self.params.min_output, self.params.max_output)
        
        if self.params.ki != 0.0 and clipped != output:
            self.integral -= (output - clipped) / self.params.ki
            
        return clipped

    def _store_debug_data(self, **kwargs):
        """Manage debug data storage with rolling buffer."""
        if len(self.debug_data) >= self.max_debug_samples:
            self.debug_data.pop(0)
            
        self.debug_data.append(PIDDebugData(
            timestamp=time.time(),
            compute_time=time.time() - self.last_time.nanoseconds / 1e9,
            **kwargs
        ))

class ThreadedGapFollower(Node):
    """Autonomous navigation node using gap detection and PID control."""

    def __init__(self):
        super().__init__('gap_follower')
        
        # Enable dynamic parameter updates
        self.add_on_set_parameters_callback(self._on_parameter_event)
        
        # State variables
        self.current_speed = 0.0
        self.latest_scan = None
        self.latest_scan_data = None
        
        self._load_parameters()
        self._init_controllers()
        self._setup_ros_components()
        
        # Register dynamic parameter callback after loading parameters
        self.get_logger().info('Node initialized with parameters: %s' % self.params)
        
        # Use ROS2 timers for scan processing and control loops
        self.callback_group_scan = ReentrantCallbackGroup()
        self.callback_group_control = ReentrantCallbackGroup()
        scan_rate = 1.0 / 100.0  # 100Hz
        control_rate = 1.0 / 100.0  # 100Hz
        self.create_timer(scan_rate, self._on_scan_timer, callback_group=self.callback_group_scan)
        self.create_timer(control_rate, self._on_control_timer, callback_group=self.callback_group_control)

    def _load_parameters(self):
        """Load and validate node parameters for autonomous navigation."""
        param_definitions = [
            # Maximum range for valid laser scan readings in meters
            # Readings beyond this range are discarded to ensure reliable obstacle detection
            ('max_range', 10.0),
            
            # Minimum width threshold for navigable gaps in meters
            # Gaps narrower than this are considered unsafe and ignored
            # Should be set wider than robot width plus safety margin
            ('min_gap_width', 0.8),
            
            # Maximum allowed discontinuity between consecutive scan points in meters
            # Used to detect potential gaps - larger values allow detecting wider openings
            # but may miss smaller obstacles
            ('max_gap_depth', 5.0),
            
            # Maximum velocity limit in meters/second
            # Upper bound on robot speed for safety
            # Should account for mechanical limits and stopping distance
            ('max_speed', 0.50),
            
            # Minimum velocity threshold in meters/second 
            # Lower bound to maintain momentum and prevent stalling
            # Especially important for differential drive robots
            ('min_speed', 0.1),
            
            # Maximum steering angle in radians
            # Physical limit of steering mechanism
            # Affects turning radius and maneuverability
            ('max_steering_angle', 0.7),
            
            # Weight factor (0-1) balancing distance vs width in gap selection
            # Higher values favor gaps closer to current heading
            # Lower values prioritize wider gaps regardless of direction
            ('distance_weight', 0.5),
            
            # Steering PID controller parameters
            # P: Direct proportional response to heading error
            # I: Accumulates and corrects steady-state error
            # D: Dampens oscillations and improves stability
            ('steering_pid.p', 2.2),
            ('steering_pid.i', 0.01),
            ('steering_pid.d', 0.3),
            
            # Speed PID controller parameters  
            # P: Primary speed error correction
            # I: Compensates for inclines and friction
            # D: Smooths acceleration/deceleration
            ('speed_pid.p', 0.8),
            ('speed_pid.i', 0.05), 
            ('speed_pid.d', 0.1),
            
            # Window size for median filter preprocessing
            # Reduces sensor noise and spurious readings
            # Larger values increase smoothing but add latency
            ('median_filter_size', 5),
            
            # Debug verbosity level (0-2)
            # 0: Errors only
            # 1: Warnings and important info
            # 2: Detailed debugging data
            ('log_level', 0),
            
            # Minimum obstacle clearance in meters
            # Critical safety parameter for collision avoidance
            # Should exceed robot radius + uncertainty margin
            ('safety_margin', 0.5)
        ]
        
        try:
            self.declare_parameters(namespace='', parameters=param_definitions)
            self.params = {name: self.get_parameter(name).value 
                          for name, _ in param_definitions}
            
            if self.params['min_speed'] > self.params['max_speed']:
                raise ValueError("Minimum speed cannot exceed maximum speed")
                
            if self.params['safety_margin'] < 0.1:
                raise ValueError("Safety margin too small (minimum 0.1m)")

        except Exception as e:
            self.get_logger().error(f"Parameter loading error: {str(e)}")
            raise

    def _init_controllers(self):
        """Initialize PID controllers with configured parameters."""
        steering_params = PIDParameters(
            kp=self.params['steering_pid.p'],
            ki=self.params['steering_pid.i'],
            kd=self.params['steering_pid.d'],
            min_output=-self.params['max_steering_angle'],
            max_output=self.params['max_steering_angle']
        )
        
        speed_params = PIDParameters(
            kp=self.params['speed_pid.p'],
            ki=self.params['speed_pid.i'],
            kd=self.params['speed_pid.d'],
            min_output=self.params['min_speed'],
            max_output=self.params['max_speed']
        )
        
        print("[DEBUG] Initializing PID controllers with steering params:", steering_params)
        self.steering_pid = PIDController(steering_params)
        self.speed_pid = PIDController(speed_params)

    def _setup_ros_components(self):
        """Configure ROS2 subscriptions and publishers."""
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, qos_profile=qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_callback, qos_profile=qos_profile_sensor_data
        )
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )
        self.debug_pub = self.create_publisher(
            Float32MultiArray, '/pid_debug', 10
        )

    def _odom_callback(self, msg: Odometry):
        """Update current vehicle speed from odometry."""
        self.current_speed = msg.twist.twist.linear.x

    def _scan_callback(self, msg: LaserScan):
        """Store the latest scan message for timer-based processing."""
        self.latest_scan = msg

    def _on_scan_timer(self):
        """Timer callback to process the latest scan message."""
        if self.latest_scan is None:
            return
        try:
            filtered_ranges = self._preprocess_scan(self.latest_scan)
            points = self._convert_to_cartesian(filtered_ranges, self.latest_scan)
            gaps = self._detect_gaps(points, self.latest_scan.angle_increment)
            if gaps:
                best_gap = self._select_optimal_gap(gaps)
                self.latest_scan_data = ScanData(
                    timestamp=self.get_clock().now().nanoseconds / 1e9,
                    target_angle=best_gap['angle'],
                    ranges=filtered_ranges,
                )
            else:
                self.latest_scan_data = ScanData(
                    timestamp=self.get_clock().now().nanoseconds / 1e9,
                    target_angle=0.0,
                    ranges=filtered_ranges,
                    is_valid=False,
                )
        except Exception as e:
            self.get_logger().error(f"Scan processing error: {str(e)}")

    def _preprocess_scan(self, msg: LaserScan) -> np.ndarray:
        """Clean and filter laser range data."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        
        valid_mask = ~(np.isnan(ranges) | np.isinf(ranges))
        ranges[~valid_mask] = self.params['max_range']
        
        if self.params['median_filter_size'] > 1:
            ranges = median_filter(
                ranges, 
                size=self.params['median_filter_size']
            )
            
        return ranges

    def _convert_to_cartesian(self, ranges: np.ndarray, msg: LaserScan) -> np.ndarray:
        """Convert polar coordinates to Cartesian points."""
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.column_stack((x, y))

    def _detect_gaps(self, points: np.ndarray, angle_inc: float) -> List[Dict]:
        """Identify navigable gaps in the environment."""
        print("[DEBUG] Starting gap detection")
        diffs = np.diff(points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        gap_indices = np.where(distances > self.params['max_gap_depth'])[0]
        
        gaps = []
        for idx in gap_indices:
            if idx + 1 >= len(points):
                continue
                
            p1 = points[idx]
            p2 = points[idx + 1]
            midpoint = (p1 + p2) / 2
            gap_width = np.linalg.norm(p2 - p1)
            
            # Calculate angle relative to robot's heading
            angle = np.arctan2(midpoint[1], midpoint[0])
            print(f"[DEBUG] Found potential gap: width={gap_width:.2f}, angle={math.degrees(angle):.2f}°")
            
            gap = {
                'start_idx': idx,
                'end_idx': idx + 1,
                'width': gap_width,
                'angle': angle,
                'distance': np.linalg.norm(midpoint),
                'midpoint': midpoint,
                'score': self._calculate_gap_score(gap_width, midpoint)
            }
            
            if self._is_valid_gap(gap):
                gaps.append(gap)
                
        if gaps:
            print(f"[DEBUG] Found {len(gaps)} valid gaps")
        else:
            print("[DEBUG] No valid gaps found")
                
        return sorted(gaps, key=lambda x: x['score'], reverse=True)


    def _calculate_gap_score(self, width: float, midpoint: np.ndarray) -> float:
        """Calculate combined score for gap quality assessment."""
        distance_score = (1 - (midpoint[0] / self.params['max_range']))
        width_score = width / self.params['max_gap_depth']
        return (self.params['distance_weight'] * distance_score +
                (1 - self.params['distance_weight']) * width_score)

    def _is_valid_gap(self, gap: Dict) -> bool:
        """Validate gap against safety criteria."""
        return (gap['width'] >= self.params['min_gap_width'] and
                gap['midpoint'][0] > self.params['safety_margin'])

    def _select_optimal_gap(self, gaps: List[Dict]) -> Dict:
        """Select the best navigable gap from detected options."""
        return gaps[0]

    def _on_control_timer(self):
        """Timer callback to compute and publish drive commands based on processed scan data."""
        if not hasattr(self, 'latest_scan_data') or self.latest_scan_data is None:
            return
        try:
            steering = self._compute_steering_command(self.latest_scan_data.target_angle)
            speed = self._compute_speed_command(steering)
            self._publish_control_command(steering, speed)
        except Exception as e:
            self.get_logger().error(f"Control loop error: {str(e)}")

    def _compute_steering_command(self, target_angle: float) -> float:
        """Calculate steering angle using PID controller."""
        steering_cmd = self.steering_pid.compute(target_angle, self.get_clock().now())
        print(f"[DEBUG] Steering command: target_angle={math.degrees(target_angle):.2f}°, cmd={math.degrees(steering_cmd):.2f}°")
        return steering_cmd

    def _compute_speed_command(self, steering: float) -> float:
        """Calculate speed command based on steering angle."""
        target_speed = self.params['max_speed'] * (
            1 - abs(steering) / self.params['max_steering_angle']
        )
        return self.speed_pid.compute(
            target_speed - self.current_speed,
            self.get_clock().now()
        )

    def _publish_control_command(self, steering: float, speed: float):
            """Publish drive command with safety checks."""
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            
            cmd.drive.steering_angle = np.clip(
                steering,
                -self.params['max_steering_angle'],
                self.params['max_steering_angle']
            )
            
            cmd.drive.speed = np.clip(
                speed,
                self.params['min_speed'],
                self.params['max_speed']
            )
            
            self.drive_pub.publish(cmd)
            self._publish_debug_info()

    def _publish_debug_info(self):
        """Publish PID controller debug information."""
        if not self.steering_pid.debug_data:
            return
            
        latest = self.steering_pid.debug_data[-1]
        debug_msg = Float32MultiArray()
        debug_msg.data = [
            latest.error,
            latest.p_term,
            latest.i_term,
            latest.d_term,
            latest.output
        ]
        self.debug_pub.publish(debug_msg)

    def destroy_node(self):
        """Clean shutdown."""
        super().destroy_node()

    def _on_parameter_event(self, params: List[rcl_interfaces.msg.Parameter]):
        """Callback for dynamic parameter updates."""
        for param in params:
            if param.name == 'max_range':
                self.params['max_range'] = param.value
            elif param.name == 'min_gap_width':
                self.params['min_gap_width'] = param.value
            elif param.name == 'max_gap_depth':
                self.params['max_gap_depth'] = param.value
            elif param.name == 'max_speed':
                self.params['max_speed'] = param.value
            elif param.name == 'min_speed':
                self.params['min_speed'] = param.value
            elif param.name == 'max_steering_angle':
                self.params['max_steering_angle'] = param.value
            elif param.name == 'distance_weight':
                self.params['distance_weight'] = param.value
            elif param.name == 'median_filter_size':
                self.params['median_filter_size'] = param.value
            elif param.name == 'log_level':
                self.params['log_level'] = param.value
            elif param.name == 'safety_margin':
                self.params['safety_margin'] = param.value

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ThreadedGapFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()