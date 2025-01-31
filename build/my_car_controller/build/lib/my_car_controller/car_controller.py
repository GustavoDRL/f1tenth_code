#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.logging import LoggingSeverity
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import ompl.base as ob
import ompl.geometric as og
import math
from tf2_ros import TransformListener, Buffer
from rcl_interfaces.msg import Parameter, ParameterDescriptor
from enum import Enum, auto
from sklearn.cluster import DBSCAN
from functools import lru_cache

class ControllerState(Enum):
    """Estados possíveis do controlador"""
    INITIALIZING = auto()
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    EMERGENCY_STOP = auto()
    ERROR = auto()

class SafetyLevel(Enum):
    """Níveis de segurança do sistema"""
    NORMAL = auto()
    CAUTION = auto()
    DANGER = auto()

class CarController(LifecycleNode):
    def __init__(self):
        super().__init__('car_controller')
        
        # Configuração inicial
        self.state = ControllerState.INITIALIZING
        self.safety_level = SafetyLevel.NORMAL
        
        # Cache para cálculos frequentes
        self.cached_scan_processing = lru_cache(maxsize=32)(self._process_scan_data_impl)
        
        # Configuração de parâmetros
        self.declare_parameters()
        self.setup_diagnostics()
        
        # Configuração de QoS
        self.setup_qos_profiles()
        
        # Inicialização de variáveis
        self.initialize_variables()
        
    def declare_parameters(self):
        """Declaração de parâmetros com descrições e validação"""
        param_descriptors = {
            'car_radius': ParameterDescriptor(
                description='Raio do veículo em metros',
                floating_point_range=[{'from_value': 0.1, 'to_value': 1.0, 'step': 0.1}]
            ),
            'max_speed': ParameterDescriptor(
                description='Velocidade máxima em m/s',
                floating_point_range=[{'from_value': 0.5, 'to_value': 5.0, 'step': 0.1}]
            ),
            # ... outros parâmetros ...
        }
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_radius', 0.3, param_descriptors['car_radius']),
                ('max_speed', 1.5, param_descriptors['max_speed']),
                ('min_speed', 0.5),
                ('max_steering_angle', 0.4),
                ('steering_smoothing', 0.3),
                ('planning_frequency', 10.0),
                ('planning_time', 0.2),
                ('goal_distance', 2.0),
                ('space_bounds', 5.0),
                ('obstacle_margin', 1.5),
                ('safety_distance', 1.0),
                ('clustering_epsilon', 0.3),
                ('min_cluster_size', 3)
            ]
        )
        
    def setup_qos_profiles(self):
        """Configuração de perfis QoS específicos para cada tipo de mensagem"""
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
    def initialize_variables(self):
        """Inicialização de variáveis internas"""
        self.latest_scan = None
        self.current_pose = None
        self.previous_steering = 0.0
        self.scan_points = None
        self.planner = None
        self.path_cache = None
        self.last_valid_path = None
        self.emergency_stop_count = 0
        
    def setup_diagnostics(self):
        """Configuração do sistema de diagnóstico"""
        self.diagnostics = {
            'planning_success_rate': 0.0,
            'average_planning_time': 0.0,
            'num_planning_attempts': 0,
            'num_successful_plans': 0,
            'last_planning_time': None,
            'average_path_length': 0.0,
            'average_clearance': 0.0,
            'emergency_stops': 0,
            'current_state': self.state.name,
            'safety_level': self.safety_level.name
        }
        
    def on_configure(self, state):
        """Callback de configuração do Lifecycle Node"""
        try:
            # Criar subscribers com QoS apropriado
            self.scan_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                self.sensor_qos
            )
            
            self.odom_sub = self.create_subscription(
                Odometry,
                '/ego_racecar/odom',
                self.odom_callback,
                self.sensor_qos
            )
            
            # Criar publishers
            self.drive_pub = self.create_publisher(
                AckermannDriveStamped,
                '/drive',
                self.control_qos
            )
            
            self.path_pub = self.create_publisher(
                Path,
                '/planned_path',
                self.control_qos
            )
            
            self.diag_pub = self.create_publisher(
                DiagnosticArray,
                '/diagnostics',
                10
            )
            
            # Configurar OMPL
            self.setup_ompl_planner()
            
            # Configurar timers
            self.planning_timer = self.create_timer(
                1.0/self.get_parameter('planning_frequency').value,
                self.planning_loop
            )
            
            self.diagnostics_timer = self.create_timer(
                1.0,
                self.publish_diagnostics
            )
            
            self.state = ControllerState.IDLE
            return rclpy.lifecycle.node.TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {str(e)}')
            return rclpy.lifecycle.node.TransitionCallbackReturn.ERROR
            
    def setup_ompl_planner(self):
        """Configuração otimizada do planejador OMPL"""
        try:
            self.space = ob.SE2StateSpace()
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(-self.get_parameter('space_bounds').value)
            bounds.setHigh(self.get_parameter('space_bounds').value)
            self.space.setBounds(bounds)
            
            self.ss = og.SimpleSetup(self.space)
            self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
            
            # Configuração avançada do RRT*
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
            
            # Ajustes dinâmicos
            workspace_size = self.get_parameter('space_bounds').value * 2
            self.planner.setRange(workspace_size * 0.1)
            self.planner.setGoalBias(0.3)
            self.planner.setKNearest(True)
            self.planner.setDelayOptimization(False)
            
            # Objetivo multi-critério
            si = self.ss.getSpaceInformation()
            opt = ob.MultiOptimizationObjective(si)
            
            pathLength = ob.PathLengthOptimizationObjective(si)
            pathLength.setCostThreshold(ob.Cost(1.5))
            opt.addObjective(pathLength, 0.7)
            
            clearance = ob.MinimaxObjective(si)
            opt.addObjective(clearance, 0.3)
            
            self.ss.setOptimizationObjective(opt)
            self.ss.setPlanner(self.planner)
            self.ss.setup()
            
        except Exception as e:
            self.get_logger().error(f'OMPL setup failed: {str(e)}')
            raise
            
    def _process_scan_data_impl(self, scan_msg):
        """Implementação do processamento de scan (usado pelo cache)"""
        try:
            angles = np.linspace(
                scan_msg.angle_min,
                scan_msg.angle_max,
                len(scan_msg.ranges),
                dtype=np.float32
            )
            
            ranges = np.array(scan_msg.ranges, dtype=np.float32)
            
            # Filtros de qualidade
            valid_mask = np.isfinite(ranges) & \
                        (ranges >= scan_msg.range_min) & \
                        (ranges <= scan_msg.range_max)
            
            if not np.any(valid_mask):
                self.get_logger().warn('No valid LIDAR points detected')
                return None
                
            # Redução de ruído
            window_size = 3
            ranges = np.convolve(ranges, np.ones(window_size)/window_size, mode='same')
            
            # Detecção de descontinuidades
            range_diff = np.abs(np.diff(ranges))
            discontinuity_threshold = 0.1
            discontinuities = range_diff > discontinuity_threshold
            
            # Conversão para coordenadas cartesianas
            x = ranges[valid_mask] * np.cos(angles[valid_mask])
            y = ranges[valid_mask] * np.sin(angles[valid_mask])
            
            # Clustering
            points = np.column_stack((x, y))
            clustering = DBSCAN(
                eps=self.get_parameter('clustering_epsilon').value,
                min_samples=self.get_parameter('min_cluster_size').value
            ).fit(points)
            
            return {
                'x': x,
                'y': y,
                'clusters': clustering.labels_,
                'discontinuities': discontinuities,
                'num_valid_points': np.sum(valid_mask),
                'mean_range': np.mean(ranges[valid_mask]),
                'max_range': np.max(ranges[valid_mask])
            }
            
        except Exception as e:
            self.get_logger().error(f'Scan processing error: {str(e)}')
            return None
            
    def process_scan_data(self):
        """Wrapper para processamento de scan com cache"""
        if self.latest_scan is None:
            return None
        return self.cached_scan_processing(self.latest_scan)
        
    def update_safety_level(self):
        """Atualização do nível de segurança baseado nas condições atuais"""
        if self.scan_points is None:
            return
            
        min_distance = np.min(np.hypot(self.scan_points['x'], self.scan_points['y']))
        safety_distance = self.get_parameter('safety_distance').value
        
        if min_distance < safety_distance * 0.5:
            self.safety_level = SafetyLevel.DANGER
        elif min_distance < safety_distance:
            self.safety_level = SafetyLevel.CAUTION
        else:
            self.safety_level = SafetyLevel.NORMAL
            
    def scan_callback(self, msg):
        """Callback para dados do LIDAR com processamento de segurança"""
        self.latest_scan = msg
        self.scan_points = self.process_scan_data()
        self.update_safety_level()
        
        if self.safety_level == SafetyLevel.DANGER:
            self.trigger_emergency_stop("Obstáculo muito próximo detectado")
            
    def trigger_emergency_stop(self, reason):
        """Ativação do modo de parada de emergência"""
        self.state = ControllerState.EMERGENCY_STOP
        self.emergency_stop_count += 1
        self.get_logger().warn(f'Emergency stop triggered: {reason}')
        self.publish_stop_command()
        
    def is_state_valid(self, state):
        """Verificação de estado com considerações de segurança aprimoradas"""
        if self.scan_points is None or self.current_pose is None:
            return False
            
        try:
            state_arr = np.array([state[0], state[1]])
            
            # Transformação para coordenadas globais
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            cos_yaw = np.cos(current_yaw)
            sin_yaw = np.sin(current_yaw)
            
            global_obstacles_x = (self.scan_points['x'] * cos_yaw - 
                                self.scan_points['y'] * sin_yaw + 
                                self.current_pose.position.x)
            global_obstacles_y = (self.scan_points['x'] * sin_yaw + 
                                self.scan_points['y'] * cos_yaw + 
                                self.current_pose.position.y)
            
            obstacles = np.vstack((global_obstacles_x, global_obstacles_y)).T
            
            # Verificação de distância com margem de segurança dinâmica
            distances = np.linalg.norm(obstacles - state_arr, axis=1)
            safety_margin = self.get_parameter('car_radius').value * \
                          self.get_parameter('obstacle_margin').value
                          
            return np.all(distances >= safety_margin)
            
        except Exception as e:
            self.get_logger().error(f'State validation error: {str(e)}')
            return False
            
    def planning_loop(self):
        """Loop principal de planejamento com gestão de estados"""
        if self.state == ControllerState.EMERGENCY_STOP:
            return
            
        if self.current_pose is None or self.scan_points is None:
            return
            
        self.state = ControllerState.PLANNING
        start_time = self.get_clock().now()
        
        try:
            start = self.space.allocState()
            goal = None
            
            # Configuração do estado inicial
            start[0] = self.current_pose.position.x
            start[1] = self.current_pose.position.y
            start[2] = self.get_yaw_from_quaternion(self.current_pose.orientation)
            
            # Obtenção do objetivo
            goal = self.get_goal_state()
            if goal is None:
                self.space.freeState(start)
                self.update_diagnostics(False, 0.0)
                self.publish_stop_command()
                return
                
            # Configuração do planejamento
            self.ss.setStartAndGoalStates(start, goal)
            planning_time = self.get_parameter('planning_time').value
            solved = self.ss.solve(planning_time)
            
            # Cálculo do tempo de planejamento
            end_time = self.get_clock().now()
            planning_duration = (end_time - start_time).nanoseconds / 1e9
            
            if solved:
                self.state = ControllerState.EXECUTING
                self.ss.simplifySolution()
                path = self.ss.getSolutionPath()
                
                # Cache do último caminho válido
                self.last_valid_path = path
                
                # Cálculo do comprimento do caminho para diagnóstico
                path_length = self.calculate_path_length(path)
                self.update_diagnostics(True, planning_duration, path_length)
                
                # Publicação do caminho e comandos
                self.publish_path(path)
                self.publish_drive_command(path)
            else:
                self.get_logger().warn('No solution found, using fallback strategy')
                self.handle_planning_failure()
                
        except Exception as e:
            self.get_logger().error(f'Planning error: {str(e)}')
            self.state = ControllerState.ERROR
            self.publish_stop_command()
        finally:
            # Limpeza de recursos
            if start:
                self.space.freeState(start)
            if goal:
                self.space.freeState(goal)

    def calculate_path_length(self, path):
        """Calcula o comprimento total do caminho"""
        total_length = 0.0
        for i in range(path.getStateCount() - 1):
            s1 = path.getState(i)
            s2 = path.getState(i + 1)
            dx = s2[0] - s1[0]
            dy = s2[1] - s1[1]
            total_length += math.sqrt(dx*dx + dy*dy)
        return total_length

    def handle_planning_failure(self):
        """Estratégia de fallback para falhas no planejamento"""
        if self.last_valid_path is not None:
            # Tentar reutilizar último caminho válido com adaptações
            self.get_logger().info('Attempting to reuse last valid path')
            modified_path = self.adapt_previous_path(self.last_valid_path)
            if modified_path:
                self.publish_path(modified_path)
                self.publish_drive_command(modified_path)
                return
                
        # Se não houver caminho anterior ou adaptação falhar
        self.trigger_emergency_stop("Planning failure with no valid fallback")

    def adapt_previous_path(self, previous_path):
        """Adapta o último caminho válido para a situação atual"""
        try:
            # Verificar se o caminho anterior ainda é válido
            for i in range(previous_path.getStateCount()):
                state = previous_path.getState(i)
                if not self.is_state_valid(state):
                    return None
                    
            # Adaptar o início do caminho para a posição atual
            adapted_path = previous_path
            start_state = adapted_path.getState(0)
            start_state[0] = self.current_pose.position.x
            start_state[1] = self.current_pose.position.y
            start_state[2] = self.get_yaw_from_quaternion(self.current_pose.orientation)
            
            return adapted_path
        except Exception as e:
            self.get_logger().error(f'Path adaptation failed: {str(e)}')
            return None

    def publish_path(self, path):
        """Publicação do caminho planejado com visualização melhorada"""
        try:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            prev_point = None
            min_distance = 0.1  # Mínima distância entre pontos para reduzir redundância
            
            for i in range(path.getStateCount()):
                state = path.getState(i)
                
                # Redução de densidade de pontos
                if prev_point is not None:
                    dx = state[0] - prev_point.x
                    dy = state[1] - prev_point.y
                    if math.sqrt(dx*dx + dy*dy) < min_distance:
                        continue
                
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = state[0]
                pose.pose.position.y = state[1]
                
                # Cálculo da orientação suave
                if i < path.getStateCount() - 1:
                    next_state = path.getState(i + 1)
                    yaw = math.atan2(next_state[1] - state[1],
                                   next_state[0] - state[0])
                else:
                    yaw = state[2]
                
                quat = Quaternion()
                quat.z = math.sin(yaw/2.0)
                quat.w = math.cos(yaw/2.0)
                pose.pose.orientation = quat
                
                path_msg.poses.append(pose)
                prev_point = pose.pose.position
                
            self.path_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().error(f'Path publishing error: {str(e)}')

    def publish_drive_command(self, path):
        """Geração e publicação de comandos de controle suaves"""
        try:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            
            if path.getStateCount() > 1:
                # Horizonte de previsão adaptativo
                horizon = min(5, path.getStateCount())
                steering_angles = []
                distances = []
                
                # Coleta de informações do caminho
                for i in range(horizon-1):
                    current = path.getState(i)
                    next_state = path.getState(i+1)
                    
                    dx = next_state[0] - current[0] 
                    dy = next_state[1] - current[1]
                    
                    distance = math.sqrt(dx*dx + dy*dy)
                    angle = math.atan2(dy, dx)
                    
                    steering_angles.append(self.normalize_angle(angle - current[2]))
                    distances.append(distance)
                
                # Pesos adaptativos baseados na distância
                weights = np.exp(-0.5 * np.array(distances))
                weights /= np.sum(weights)
                
                # Cálculo do ângulo de direção suavizado
                raw_steering = np.average(steering_angles, weights=weights)
                steering_angle = (self.steering_smoothing * raw_steering + 
                                (1 - self.steering_smoothing) * self.previous_steering)
                
                # Limitação do ângulo de direção
                steering_angle = np.clip(
                    steering_angle,
                    -self.max_steering_angle,
                    self.max_steering_angle
                )
                
                # Velocidade adaptativa baseada em múltiplos fatores
                base_speed_factor = 1.0 - (abs(steering_angle) / self.max_steering_angle) ** 2
                safety_factor = self.get_safety_factor()
                
                speed = self.min_speed + (self.max_speed - self.min_speed) * \
                        base_speed_factor * safety_factor
                
                msg.drive.steering_angle = float(steering_angle)
                msg.drive.speed = float(speed)
                
                self.previous_steering = steering_angle
                
            self.drive_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Drive command error: {str(e)}')
            self.publish_stop_command()

    def get_safety_factor(self):
        """Calcula fator de segurança baseado nas condições do ambiente"""
        if self.safety_level == SafetyLevel.DANGER:
            return 0.0
        elif self.safety_level == SafetyLevel.CAUTION:
            return 0.5
        else:
            return 1.0

    def publish_diagnostics(self):
        """Publicação de informações de diagnóstico detalhadas"""
        try:
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Status geral do sistema
            status = DiagnosticStatus()
            status.name = "CarController"
            status.hardware_id = "autonomous_vehicle"
            
            # Determinação do nível de diagnóstico
            if self.state == ControllerState.ERROR:
                status.level = DiagnosticStatus.ERROR
                status.message = "System in error state"
            elif self.state == ControllerState.EMERGENCY_STOP:
                status.level = DiagnosticStatus.WARN
                status.message = "Emergency stop active"
            elif self.diagnostics['planning_success_rate'] > 0.8:
                status.level = DiagnosticStatus.OK
                status.message = "System operating normally"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = "Reduced planning performance"
                
            # Adição de valores de diagnóstico
            for key, value in self.diagnostics.items():
                if value is not None:
                    kv = KeyValue()
                    kv.key = key
                    kv.value = str(value)
                    status.values.append(kv)
                    
            msg.status.append(status)
            self.diag_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Diagnostics publishing error: {str(e)}')

def main(args=None):
    """Função principal com gestão adequada de recursos"""
    rclpy.init(args=args)
    controller = None
    
    try:
        controller = CarController()
        # Transição para estado ativo
        controller.trigger_transition(rclpy.lifecycle.node.Transition.TRANSITION_CONFIGURE)
        controller.trigger_transition(rclpy.lifecycle.node.Transition.TRANSITION_ACTIVATE)
        
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(controller)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            
    except Exception as e:
        print(f'Critical error in main: {str(e)}')
    finally:
        if controller:
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()