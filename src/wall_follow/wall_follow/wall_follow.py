#!/usr/bin/env python3
"""
Wall Following Algorithm - Versão Híbrida Definitiva
Combina o melhor das duas implementações:
- QoS compatível com LiDAR
- Busca robusta de dados válidos
- PID otimizado e adaptativo
- Controle de velocidade inteligente
- Sistema de segurança multicamadas
- Debug detalhado
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
import math
import time
import numpy as np

class WallFollowHybrid(Node):

    def __init__(self):
        super().__init__('wall_follow_hybrid')
        
        # Configuração do matplotlib para plotting em tempo real
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Plot do erro
        self.error_line, = self.ax1.plot([], [], 'b-', label='Error')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Error (m)')
        self.ax1.set_title('Wall Following Error')
        self.ax1.grid(True)
        self.ax1.legend()
        
        # Plot da velocidade e ângulo
        self.speed_line, = self.ax2.plot([], [], 'g-', label='Speed')
        self.angle_line, = self.ax2.plot([], [], 'r-', label='Steering Angle')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Value')
        self.ax2.set_title('Control Signals')
        self.ax2.grid(True)
        self.ax2.legend()

        # QoS Profile compatível com LiDAR (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber do LiDAR com QoS compatível
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Publisher para comandos Ackermann
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # ==================== PARÂMETROS DE CONTROLE ====================
        
        # PID Parameters - Otimizados para estabilidade
        self.kp = 8.0        # Proporcional - reduzido para suavidade
        self.ki = 0.001      # Integral - baixo para evitar windup
        self.kd = 0.05       # Derivativo - suave para estabilidade
        
        # Limites de controle
        self.max_steering_angle = 25.0  # graus - limite físico do servo
        self.integral_limit = 10.0      # anti-windup
        
        # ==================== PARÂMETROS GEOMÉTRICOS ====================
        
        # Ângulos do LiDAR - otimizados para S2PRO
        self.angleA = 60    # Primeira medição (60° = lateral direita)
        self.angleB = 90    # Segunda medição (90° = perpendicular)
        self.search_range = 15.0  # Faixa de busca em graus para dados válidos
        
        # Distância desejada da parede
        self.target_distance = 0.8  # metros
        
        # ==================== CONTROLE DE VELOCIDADE ====================
        
        self.base_speed = 0.8
        self.max_speed = 1.5
        self.min_speed = 0.3
        self.cruise_speed = 1.2
        
        # Zonas de controle de velocidade
        self.precision_zone = 0.1   # Muito perto - acelerar
        self.comfort_zone = 0.3     # Zona confortável - velocidade normal
        self.caution_zone = 0.6     # Zona de cuidado - reduzir velocidade
        
        # ==================== FILTROS E HISTÓRICO ====================
        
        # Histórico para controle
        self.integral = 0.0
        self.prev_error = 0.0
        self.error_history = []
        self.time_history = []
        self.speed_history = []
        self.angle_history = []
        
        # Filtro de média móvel para estabilizar leituras
        self.error_buffer = []
        self.buffer_size = 5
        
        # ==================== SISTEMA DE SEGURANÇA ====================
        
        # Contadores de falha
        self.consecutive_failures = 0
        self.max_failures = 5
        self.last_valid_error = 0.0
        
        # Limites de segurança
        self.min_valid_distance = 0.15  # metros
        self.max_valid_distance = 5.0   # metros
        
        # ==================== INICIALIZAÇÃO ====================
        
        self.start_time = time.time()
        self.node_ready = True
        
        print("=" * 60)
        print("🚀 WALL FOLLOW HYBRID - VERSÃO DEFINITIVA")
        print("=" * 60)
        print(f"📐 Ângulos: {self.angleA}°/{self.angleB}° (±{self.search_range}°)")
        print(f"📏 Distância alvo: {self.target_distance}m")
        print(f"⚙️  PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        print(f"🚗 Velocidades: {self.min_speed}-{self.max_speed} m/s")
        print("✅ Sistema inicializado e pronto!")
        print("=" * 60)

    def get_range_by_angle_robust(self, scan: LaserScan, target_angle: float, search_range: float = None) -> float:
        """
        Busca robusta por dados válidos em uma faixa ao redor do ângulo alvo.
        
        Args:
            scan: Dados do LiDAR
            target_angle: Ângulo alvo em graus
            search_range: Faixa de busca em graus (padrão: self.search_range)
        
        Returns:
            float: Distância válida ou None se não encontrar
        """
        if search_range is None:
            search_range = self.search_range
            
        target_rad = math.radians(target_angle)
        search_rad = math.radians(search_range)
        
        valid_readings = []
        
        # Busca em uma faixa ao redor do ângulo alvo
        search_steps = int(search_range * 2)  # 1° por passo
        for step in range(-search_steps, search_steps + 1):
            angle_offset = math.radians(step * 0.5)  # 0.5° por passo
            test_angle = target_rad + angle_offset
            
            # Verifica limites do scan
            if test_angle < scan.angle_min or test_angle > scan.angle_max:
                continue
                
            # Calcula índice no array
            index = int((test_angle - scan.angle_min) / scan.angle_increment)
            
            # Verifica se índice é válido
            if 0 <= index < len(scan.ranges):
                range_value = scan.ranges[index]
                
                # Validação rigorosa da leitura
                if (not math.isnan(range_value) and 
                    not math.isinf(range_value) and 
                    self.min_valid_distance <= range_value <= self.max_valid_distance):
                    valid_readings.append(range_value)
        
        # Processa leituras válidas
        if len(valid_readings) >= 3:
            # Remove outliers (10% superior e inferior)
            valid_readings.sort()
            trim_count = max(1, len(valid_readings) // 10)
            trimmed = valid_readings[trim_count:-trim_count] if trim_count > 0 else valid_readings
            
            # Retorna mediana das leituras trimmed
            return trimmed[len(trimmed) // 2]
        elif len(valid_readings) > 0:
            # Se poucas leituras, usa mediana simples
            valid_readings.sort()
            return valid_readings[len(valid_readings) // 2]
        
        return None

    def calculate_wall_error(self, msg: LaserScan) -> float:
        """
        Calcula o erro de distância da parede usando geometria robusta.
        
        Args:
            msg: Dados do LiDAR
            
        Returns:
            float: Erro de distância (positivo = muito longe, negativo = muito perto)
        """
        # Busca dados válidos nos dois ângulos
        distance_a = self.get_range_by_angle_robust(msg, self.angleA)
        distance_b = self.get_range_by_angle_robust(msg, self.angleB)
        
        # Verifica se conseguiu dados válidos
        if distance_a is None or distance_b is None:
            print(f"⚠️  Dados inválidos - A: {distance_a}, B: {distance_b}")
            return None
        
        # Verifica se as distâncias são razoáveis
        if (distance_a < self.min_valid_distance or distance_a > self.max_valid_distance or
            distance_b < self.min_valid_distance or distance_b > self.max_valid_distance):
            print(f"⚠️  Distâncias fora do range - A: {distance_a:.3f}m, B: {distance_b:.3f}m")
            return None
        
        try:
            # Cálculo geométrico do erro
            theta = math.radians(self.angleB - self.angleA)
            
            # Proteção contra divisão por zero
            denominator = distance_a * math.sin(theta)
            if abs(denominator) < 0.001:
                print(f"⚠️  Geometria degenerada - denominador: {denominator:.6f}")
                return None
            
            # Calcula ângulo alpha e distância perpendicular
            alpha = math.atan((distance_a * math.cos(theta) - distance_b) / denominator)
            perpendicular_distance = distance_b * math.cos(alpha)
            
            # Erro = distância atual - distância desejada
            error = perpendicular_distance - self.target_distance
            
            # Debug detalhado
            print(f"📐 A={distance_a:.3f}m, B={distance_b:.3f}m, α={math.degrees(alpha):.1f}°, D⊥={perpendicular_distance:.3f}m, Erro={error:.3f}m")
            
            return error
            
        except Exception as e:
            print(f"❌ Erro no cálculo geométrico: {e}")
            return None

    def calculate_adaptive_velocity(self, error: float) -> float:
        """
        Calcula velocidade adaptativa baseada no erro e contexto.
        
        Args:
            error: Erro atual de distância
            
        Returns:
            float: Velocidade calculada
        """
        abs_error = abs(error)
        
        # Sistema de zonas de velocidade
        if abs_error <= self.precision_zone:
            # Muito próximo da trajetória ideal - pode acelerar
            velocity = self.cruise_speed
            zone = "CRUISE"
        elif abs_error <= self.comfort_zone:
            # Zona confortável - velocidade base
            velocity = self.base_speed
            zone = "COMFORT"
        elif abs_error <= self.caution_zone:
            # Precisa de correção - reduzir velocidade
            velocity = self.base_speed * 0.7
            zone = "CAUTION"
        else:
            # Muito longe - velocidade mínima para correção
            velocity = self.min_speed
            zone = "CORRECTION"
        
        # Ajuste baseado na tendência do erro
        if len(self.error_history) >= 3:
            recent_errors = self.error_history[-3:]
            error_trend = (recent_errors[-1] - recent_errors[0]) / 3
            
            # Se erro está diminuindo, pode acelerar um pouco
            if abs(error_trend) < 0.05:  # Tendência estável
                velocity *= 1.1
            elif error_trend * error < 0:  # Erro diminuindo
                velocity *= 1.05
        
        # Aplica limites finais
        velocity = max(self.min_speed, min(self.max_speed, velocity))
        
        print(f"🚗 Zona: {zone}, Velocidade: {velocity:.2f}m/s")
        return velocity

    def apply_pid_control(self, error: float) -> float:
        """
        Aplica controle PID com anti-windup e limites.
        
        Args:
            error: Erro atual
            
        Returns:
            float: Ângulo de steering em graus
        """
        # Filtro de média móvel no erro
        self.error_buffer.append(error)
        if len(self.error_buffer) > self.buffer_size:
            self.error_buffer.pop(0)
        
        filtered_error = sum(self.error_buffer) / len(self.error_buffer)
        
        # Termo proporcional
        P = self.kp * filtered_error
        
        # Termo integral com anti-windup
        self.integral += filtered_error
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        I = self.ki * self.integral
        
        # Termo derivativo
        derivative = filtered_error - self.prev_error
        D = self.kd * derivative
        self.prev_error = filtered_error
        
        # Saída PID
        pid_output = P + I + D
        
        # Aplicar limites de steering
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, pid_output))
        
        print(f"🎛️  PID: P={P:.2f}, I={I:.2f}, D={D:.2f} → {steering_angle:.2f}°")
        return steering_angle

    def publish_ackermann_command(self, velocity: float, steering_angle_deg: float):
        """
        Publica comando Ackermann com validação.
        
        Args:
            velocity: Velocidade em m/s
            steering_angle_deg: Ângulo de steering em graus
        """
        # Validação final dos comandos
        velocity = max(0.0, min(self.max_speed, velocity))
        steering_angle_deg = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle_deg))
        
        # Criar e publicar comando
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = math.radians(-steering_angle_deg)  # Inverte para convenção
        
        self.publisher.publish(cmd)
        
        print(f"📤 Comando: V={velocity:.2f}m/s, θ={steering_angle_deg:.2f}°")

    def publish_stop_command(self):
        """Publica comando de parada de emergência."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        
        self.publisher.publish(cmd)
        print("🛑 COMANDO DE PARADA ENVIADO")

    def update_plots(self):
        """Atualiza plots em tempo real."""
        if len(self.time_history) < 2:
            return
            
        try:
            # Plot do erro
            self.error_line.set_data(self.time_history, self.error_history)
            self.ax1.relim()
            self.ax1.autoscale_view()
            
            # Plot dos sinais de controle
            self.speed_line.set_data(self.time_history, self.speed_history)
            self.angle_line.set_data(self.time_history, self.angle_history)
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            # Atualiza display
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except:
            pass  # Ignora erros de plotting

    def scan_callback(self, msg: LaserScan):
        """
        Callback principal do LiDAR - processa dados e gera comandos.
        
        Args:
            msg: Mensagem do LiDAR
        """
        current_time = time.time() - self.start_time
        
        # Calcula erro de distância da parede
        error = self.calculate_wall_error(msg)
        
        if error is not None:
            # Reset contador de falhas
            self.consecutive_failures = 0
            self.last_valid_error = error
            
            # Calcula velocidade adaptativa
            velocity = self.calculate_adaptive_velocity(error)
            
            # Aplica controle PID
            steering_angle = self.apply_pid_control(error)
            
            # Publica comando
            self.publish_ackermann_command(velocity, steering_angle)
            
            # Armazena histórico
            self.error_history.append(error)
            self.time_history.append(current_time)
            self.speed_history.append(velocity)
            self.angle_history.append(steering_angle)
            
            # Limita tamanho do histórico
            max_history = 200
            if len(self.error_history) > max_history:
                self.error_history = self.error_history[-max_history:]
                self.time_history = self.time_history[-max_history:]
                self.speed_history = self.speed_history[-max_history:]
                self.angle_history = self.angle_history[-max_history:]
            
            # Atualiza plots a cada 10 iterações para performance
            if len(self.time_history) % 10 == 0:
                self.update_plots()
            
            print(f"✅ Sistema operando normalmente | Erro: {error:.3f}m")
            
        else:
            # Incrementa contador de falhas
            self.consecutive_failures += 1
            
            if self.consecutive_failures >= self.max_failures:
                # Muitas falhas consecutivas - parar por segurança
                self.publish_stop_command()
                print(f"❌ {self.consecutive_failures} falhas consecutivas - SISTEMA PARADO")
            else:
                # Usa último erro válido com velocidade reduzida
                reduced_velocity = self.min_speed
                steering_angle = self.apply_pid_control(self.last_valid_error * 0.5)  # Erro reduzido
                self.publish_ackermann_command(reduced_velocity, steering_angle)
                print(f"⚠️  Usando último erro válido ({self.last_valid_error:.3f}m) - Falha {self.consecutive_failures}/{self.max_failures}")


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    
    try:
        wall_follow_node = WallFollowHybrid()
        print("🎯 Iniciando Wall Follow Hybrid...")
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        print("\n🛑 Parando Wall Follow por solicitação do usuário...")
    except Exception as e:
        print(f"❌ Erro inesperado: {e}")
    finally:
        if 'wall_follow_node' in locals():
            wall_follow_node.destroy_node()
        rclpy.shutdown()
        print("✅ Wall Follow finalizado.")


if __name__ == '__main__':
    main()
