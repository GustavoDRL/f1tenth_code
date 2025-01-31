#!/usr/bin/env python3
"""ROS2 node for autonomous navigation using laser scan gap detection and PID control."""

import math
import time
import threading
from queue import Queue, Empty
from dataclasses import dataclass, field
from typing import List, Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy
from scipy.ndimage import median_filter

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

    def __init__(self, params: PIDParameters):
        self.params = params
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
        
        # Queues for thread communication
        self.scan_queue = Queue(maxsize=1)
        self.command_queue = Queue(maxsize=1)
        
        # Threading locks
        self.pid_lock = threading.Lock()
        self.scan_lock = threading.Lock()
        
        # State variables
        self.current_speed = 0.0
        self.should_run = True
        self.previous_gap = None
        self.gap_switch_threshold = 0.2  # 20% better score required to switch gaps
        
        self._load_parameters()
        self._init_controllers()
        self._setup_ros_components()
        
        # Start worker threads
        self.scan_thread = threading.Thread(target=self._scan_processing_loop)
        self.control_thread = threading.Thread(target=self._control_loop)
        self.scan_thread.start()
        self.control_thread.start()

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
            ('max_speed', 1.0),
            
            # Minimum velocity threshold in meters/second 
            # Lower bound to maintain momentum and prevent stalling
            # Especially important for differential drive robots
            ('min_speed', 0.5),
            
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
            print(f"Parameter loading error: {str(e)}")
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
        """Queue new scan data for processing."""
        try:
            if self.scan_queue.full():
                self.scan_queue.get_nowait()  # Remove old scan
            self.scan_queue.put_nowait(msg)
        except Exception as e:
            print(f"Scan queue error: {str(e)}")

    def _scan_processing_loop(self):
        """Dedicated thread for scan processing."""
        while self.should_run:
            try:
                scan_msg = self.scan_queue.get(timeout=0.1)
                
                with self.scan_lock:
                    filtered_ranges = self._preprocess_scan(scan_msg)
                    points = self._convert_to_cartesian(filtered_ranges, scan_msg)
                    gaps = self._detect_gaps(points, scan_msg.angle_increment)
                    
                    if gaps:
                        best_gap = self._select_optimal_gap(gaps)
                        scan_data = ScanData(
                            timestamp=self.get_clock().now().nanoseconds / 1e9,
                            target_angle=best_gap['angle'],
                            ranges=filtered_ranges
                        )
                    else:
                        scan_data = ScanData(
                            timestamp=self.get_clock().now().nanoseconds / 1e9,
                            target_angle=0.0,
                            ranges=filtered_ranges,
                            is_valid=False
                        )
                    
                    if self.command_queue.full():
                        self.command_queue.get_nowait()
                    self.command_queue.put_nowait(scan_data)
                    
            except Empty:
                continue
            except Exception as e:
                print(f"Scan processing error: {str(e)}")

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
        """Improved gap detection using region growing algorithm."""
        gaps = []
        current_gap = None
        min_points_for_gap = 3  # Minimum consecutive points to consider as a gap
        
        for i in range(1, len(points)):
            distance = np.linalg.norm(points[i] - points[i-1])
            
            if distance > self.params['max_gap_depth']:
                if current_gap is None:
                    current_gap = {'start': i-1, 'end': i, 'depths': []}
                current_gap['end'] = i
                current_gap['depths'].append(min(points[i-1][0], points[i][0]))
            else:
                if current_gap is not None:
                    # Only consider gaps with sufficient angular span
                    angular_width = (current_gap['end'] - current_gap['start']) * angle_inc
                    if angular_width >= math.radians(10):  # 10 degree minimum
                        current_gap['min_depth'] = min(current_gap['depths'])
                        gaps.append(current_gap)
                    current_gap = None

        # Process remaining gap
        if current_gap is not None:
            gaps.append(current_gap)

        return self._process_gap_candidates(gaps, points, angle_inc)

    def _process_gap_candidates(self, gaps, points, angle_inc):
        """Process raw gap candidates into validated gaps."""
        valid_gaps = []
        
        for gap in gaps:
            start_idx = gap['start']
            end_idx = gap['end']
            
            # Calculate actual gap width using law of cosines
            a = np.linalg.norm(points[start_idx])
            b = np.linalg.norm(points[end_idx])
            theta = (end_idx - start_idx) * angle_inc
            width = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(theta))
            
            # Calculate safety metrics
            safety_margin = self.params['safety_margin']
            gap_center = (points[start_idx] + points[end_idx]) / 2
            angle_to_center = math.atan2(gap_center[1], gap_center[0])
            
            # Conservative width estimate
            effective_width = width * 0.7  # Account for perception uncertainty
            
            if (effective_width >= self.params['min_gap_width'] and
                gap['min_depth'] > safety_margin):
                
                valid_gaps.append({
                    'start_idx': start_idx,
                    'end_idx': end_idx,
                    'width': effective_width,
                    'angle': angle_to_center,
                    'depth': gap['min_depth'],
                    'angular_span': theta,
                    'score': self._calculate_gap_score(effective_width, gap_center, theta)
                })
        
        return sorted(valid_gaps, key=lambda x: x['score'], reverse=True)

    def _calculate_gap_score(self, width: float, midpoint: np.ndarray, angular_span: float) -> float:
        """Enhanced scoring function with multiple factors."""
        distance_score = math.exp(-0.5 * midpoint[0]/self.params['max_range'])
        width_score = math.tanh(width / self.params['min_gap_width'])
        alignment_score = math.exp(-2 * abs(midpoint[1]/midpoint[0]))  # Prefer centered gaps
        span_score = 1 - math.exp(-angular_span/math.radians(30))
        
        return (0.4 * distance_score +
                0.3 * width_score +
                0.2 * alignment_score +
                0.1 * span_score)

    def _is_valid_gap(self, gap: Dict) -> bool:
        """Validate gap against safety criteria."""
        return (gap['width'] >= self.params['min_gap_width'] and
                gap['midpoint'][0] > self.params['safety_margin'])


    def _select_optimal_gap(self, gaps: List[Dict]) -> Dict:
        """Select best gap with hysteresis to prevent rapid switching."""
        if not gaps:
            return None
            
        best_gap = gaps[0]
        
        if self.previous_gap is not None:
            current_best_score = best_gap['score']
            previous_score = self.previous_gap['score'] * (1 + self.gap_switch_threshold)
            
            if current_best_score > previous_score:
                self.previous_gap = best_gap
            else:
                best_gap = self.previous_gap
        else:
            self.previous_gap = best_gap
        
        return best_gap

    def _control_loop(self):
        """Dedicated thread for control computation."""
        while self.should_run:
            try:
                scan_data = self.command_queue.get(timeout=0.1)
                
                if not scan_data.is_valid:
                    print("[DEBUG] Invalid scan data, executing emergency stop")
                    self._execute_emergency_stop()
                    continue
                
                with self.pid_lock:
                    steering_cmd = self._compute_steering_command(scan_data.target_angle)
                    speed_cmd = self._compute_speed_command(steering_cmd)
                    self._publish_control_command(steering_cmd, speed_cmd)
            except Empty:
                continue
            except Exception as e:
                print(f"Control loop error: {str(e)}")
                self._execute_emergency_stop()

    def _compute_steering_command(self, target_angle: float) -> float:
        """Calculate steering angle using PID controller."""
        steering_cmd = self.steering_pid.compute(target_angle, self.get_clock().now())
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

    def _execute_emergency_stop(self):
        """Immediately stop the vehicle."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        self.drive_pub.publish(cmd)

    def destroy_node(self):
        """Clean shutdown of threads."""
        self.should_run = False
        self.scan_thread.join()
        self.control_thread.join()
        super().destroy_node()

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