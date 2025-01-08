import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('gap_follow')
        
        # Parâmetros fundamentais do algoritmo
        self.BUBBLE_RADIUS = 40
        self.PREPROCESS_CONV_SIZE = 3
        self.MIN_GAP_SIZE = 15
        self.MAX_LIDAR_DIST = 3.0
        self.SPEED = 0.50
        self.THETA_SCALE = 0.20
        
        # Parâmetros de segurança e controle
        self.MAX_ANGLE_CHANGE = 0.5  # rad/s
        self.MAX_SPEED_CHANGE = 0.3  # m/s
        self.MIN_SAFE_DISTANCE = 0.5  # metros
        
        # Estado interno para controle suave
        self.last_angle = 0.0
        self.last_speed = 0.0
        self.last_process_time = time.time()
        
        # Inicialização de publishers e subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
            
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
            
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10)
        
        # Buffer circular para filtragem temporal
        self.angle_buffer = np.zeros(5)
        self.speed_buffer = np.zeros(5)
        self.buffer_idx = 0
        
        # Inicialização do sistema de diagnóstico
        self.total_processed = 0
        self.error_count = 0
        self.process_times = []

    def preprocess_lidar(self, ranges):
        """
        Pré-processamento otimizado dos dados do LIDAR com tratamento 
        robusto de valores inválidos e suavização eficiente.
        """
        try:
            # Conversão eficiente para array numpy com tipo específico
            proc_ranges = np.array(ranges, dtype=np.float32)
            
            # Tratamento vetorizado de valores inválidos
            mask_invalid = np.logical_or(np.isinf(proc_ranges), 
                                       np.isnan(proc_ranges))
            proc_ranges[mask_invalid] = self.MAX_LIDAR_DIST
            
            # Clipping vetorizado com alocação in-place
            np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST, out=proc_ranges)
            
            # Suavização otimizada com preservação de bordas
            window_size = self.PREPROCESS_CONV_SIZE
            weights = np.ones(window_size) / window_size
            
            # Convolução eficiente com modo 'valid'
            smoothed = np.convolve(proc_ranges, weights, mode='valid')
            
            # Padding inteligente preservando características das bordas
            pad_size = (window_size - 1) // 2
            proc_ranges = np.pad(smoothed, (pad_size, pad_size), mode='edge')
            
            return proc_ranges
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Erro no pré-processamento: {str(e)}')
            return None

    def find_max_gap(self, free_space_ranges):
        """
        Detecção otimizada de gaps com tratamento robusto de casos especiais
        e análise vetorizada de candidatos.
        """
        try:
            # Binarização vetorizada do espaço livre
            ranges_binary = free_space_ranges > 0
            
            # Caso especial: espaço totalmente livre
            if np.all(ranges_binary):
                return 0, len(free_space_ranges)
                
            # Caso especial: espaço totalmente ocupado
            if not np.any(ranges_binary):
                return None, None
            
            # Detecção vetorizada de transições
            transitions = np.where(np.diff(ranges_binary))[0]
            
            if len(transitions) < 2:
                return None, None
            
            # Análise vetorizada de gaps
            starts = transitions[::2]
            ends = transitions[1::2]
            
            if len(starts) == 0 or len(ends) == 0:
                return None, None
            
            # Cálculo e filtragem de gaps por tamanho e qualidade
            gap_sizes = ends - starts
            gap_qualities = np.array([
                np.mean(free_space_ranges[s:e]) for s, e in zip(starts, ends)
            ])
            
            # Ponderação entre tamanho e qualidade do gap
            gap_scores = gap_sizes * gap_qualities
            valid_gaps = gap_sizes >= self.MIN_GAP_SIZE
            
            if not np.any(valid_gaps):
                return None, None
            
            max_gap_idx = np.argmax(gap_scores * valid_gaps)
            return starts[max_gap_idx], ends[max_gap_idx]
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Erro na detecção de gaps: {str(e)}')
            return None, None

    def find_best_point(self, start_i, end_i, ranges):
        """
        Seleção otimizada do ponto objetivo considerando múltiplos critérios
        de segurança e eficiência.
        """
        try:
            gap_ranges = ranges[start_i:end_i]
            
            # Normalização das distâncias
            norm_ranges = gap_ranges / self.MAX_LIDAR_DIST
            
            # Pesos para diferentes critérios
            distance_weight = 0.4
            center_weight = 0.4
            safety_weight = 0.2
            
            # Análise de distância
            furthest_idx = np.argmax(norm_ranges)
            
            # Análise de centralidade
            center_idx = (end_i - start_i) // 2
            
            # Análise de segurança considerando vizinhança
            safety_kernel = np.array([0.3, 0.4, 0.3])
            safety_scores = np.convolve(norm_ranges, safety_kernel, mode='same')
            safest_idx = np.argmax(safety_scores)
            
            # Combinação ponderada dos critérios
            best_idx = int(
                distance_weight * furthest_idx +
                center_weight * center_idx +
                safety_weight * safest_idx
            )
            
            return start_i + best_idx
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Erro na seleção do melhor ponto: {str(e)}')
            return start_i + (end_i - start_i) // 2  # Fallback para o centro

    def check_safety_constraints(self, angle, speed):
        """
        Validação e ajuste de comandos de controle para garantir 
        operação segura e suave.
        """
        try:
            # Limitação temporal de variações
            angle = np.clip(angle,
                          self.last_angle - self.MAX_ANGLE_CHANGE,
                          self.last_angle + self.MAX_ANGLE_CHANGE)
            
            speed = np.clip(speed,
                          self.last_speed - self.MAX_SPEED_CHANGE,
                          self.last_speed + self.MAX_SPEED_CHANGE)
            
            # Atualização do buffer circular
            self.angle_buffer[self.buffer_idx] = angle
            self.speed_buffer[self.buffer_idx] = speed
            self.buffer_idx = (self.buffer_idx + 1) % len(self.angle_buffer)
            
            # Filtragem temporal usando média móvel
            angle = np.mean(self.angle_buffer)
            speed = np.mean(self.speed_buffer)
            
            # Atualização do estado
            self.last_angle = angle
            self.last_speed = speed
            
            return angle, speed
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Erro na validação de segurança: {str(e)}')
            return self.last_angle, self.last_speed

    def publish_diagnostics(self):
        """
        Publicação de informações de diagnóstico para monitoramento
        e debug do sistema.
        """
        try:
            diag_msg = DiagnosticArray()
            status = DiagnosticStatus()
            
            # Cálculo de métricas de performance
            current_time = time.time()
            process_time = current_time - self.last_process_time
            self.process_times.append(process_time)
            
            if len(self.process_times) > 100:
                self.process_times.pop(0)
            
            avg_process_time = np.mean(self.process_times)
            
            # Adição de métricas ao diagnóstico
            status.values = [
                KeyValue(key='process_time', value=str(process_time)),
                KeyValue(key='avg_process_time', value=str(avg_process_time)),
                KeyValue(key='total_processed', value=str(self.total_processed)),
                KeyValue(key='error_count', value=str(self.error_count))
            ]
            
            # Determinação do status do sistema
            if self.error_count > 10:
                status.level = DiagnosticStatus.ERROR
                status.message = "Alto número de erros detectado"
            elif avg_process_time > 0.1:  # 100ms
                status.level = DiagnosticStatus.WARN
                status.message = "Tempo de processamento elevado"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Sistema operando normalmente"
            
            diag_msg.status = [status]
            self.diagnostic_pub.publish(diag_msg)
            
        except Exception as e:
            self.get_logger().error(f'Erro na publicação de diagnósticos: {str(e)}')

    def lidar_callback(self, data):
        """
        Callback principal do sistema, integrando todos os componentes
        com tratamento robusto de erros e monitoramento de performance.
        """
        start_time = time.time()
        
        try:
            # Pré-processamento dos dados
            proc_ranges = self.preprocess_lidar(data.ranges)
            if proc_ranges is None:
                return
            
            # Identificação do ponto mais próximo
            closest_idx = np.argmin(proc_ranges)
            
            # Criação da bolha de segurança
            bubble_idxs = np.arange(
                max(0, closest_idx - self.BUBBLE_RADIUS),
                min(len(proc_ranges), closest_idx + self.BUBBLE_RADIUS)
            )
            proc_ranges[bubble_idxs] = 0
            
            # Identificação do gap máximo
            start_i, end_i = self.find_max_gap(proc_ranges)
            
            if start_i is None:
                # Comportamento defensivo quando não há gaps válidos
                angle = - (len(proc_ranges)/2 - closest_idx) / len(proc_ranges) * self.THETA_SCALE
                speed = self.SPEED * 0.5
            else:
                # Seleção do melhor ponto e cálculo do ângulo
                best_idx = self.find_best_point(start_i, end_i, proc_ranges)
                angle = - (best_idx - len(proc_ranges)/2) / len(proc_ranges) * data.angle_increment * self.THETA_SCALE
                speed = self.SPEED
            
            # Validação de segurança dos comandos
            angle, speed = self.check_safety_constraints(angle, speed)
            
            # Publicação dos comandos de controle
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = speed
            drive_msg.drive.steering_angle = angle
            self.drive_pub.publish(drive_msg)
            
            # Atualização de métricas
            self.total_processed += 1
            self.last_process_time = time.time()
            
            # Publicação de diagnósticos
            self.publish_diagnostics()
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Erro no processamento principal: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Inicializado")
    gap_follow = ReactiveFollowGap()
    rclpy.spin(gap_follow)
    gap_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()