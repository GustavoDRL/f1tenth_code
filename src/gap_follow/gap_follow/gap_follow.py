#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')
        
        # Parâmetros
        self.max_range = 10.0     # Alcance máximo do laser a ser considerado
        self.min_gap_width = 0.5  # Largura mínima do gap em metros
        self.max_gap_depth = 1.0  # Profundidade máxima para considerar um gap
        
        # Subscribers e Publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'gaps_markers',
            10)
        
        self.get_logger().info('Gap Follower node initialized')

    def find_gaps_discontinuity(self, ranges, angle_min, angle_increment):
        """
        Detecta gaps baseado em descontinuidades nas leituras do laser.
        Utiliza distância euclidiana entre pontos consecutivos.
        """
        gaps = []
        points = []  # Lista para armazenar pontos em coordenadas cartesianas
        
        # Converter valores inválidos para max_range
        ranges = [self.max_range if np.isinf(r) or np.isnan(r) else r for r in ranges]
        
        # Converter leituras para coordenadas cartesianas
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append((x, y))
        
        # Detectar gaps baseado em descontinuidades
        for i in range(1, len(points)):
            p1 = points[i-1]
            p2 = points[i]
            
            # Calcular distância euclidiana entre pontos consecutivos
            euclidean_dist = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            
            # Se a distância for maior que o limiar, é um gap
            if euclidean_dist > self.max_gap_depth:
                # Calcular o ângulo médio do gap
                angle_mid = angle_min + (i - 0.5) * angle_increment
                
                # Calcular o ponto médio do gap
                mid_x = (p1[0] + p2[0]) / 2
                mid_y = (p1[1] + p2[1]) / 2
                
                # Calcular a direção do gap (normal à linha entre os pontos)
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                gap_angle = np.arctan2(dy, dx) + np.pi/2  # Normal ao gap
                
                gaps.append({
                    'start': i-1,
                    'end': i,
                    'center': (i-1 + i) // 2,
                    'width': euclidean_dist,
                    'angle': gap_angle,
                    'mid_point': (mid_x, mid_y),
                    'confidence': euclidean_dist / self.max_gap_depth  # Medida de confiança
                })
        
        # Filtrar gaps muito pequenos
        gaps = [gap for gap in gaps if gap['width'] >= self.min_gap_width]
        
        # Ordenar gaps por largura (do maior para o menor)
        gaps.sort(key=lambda x: x['width'], reverse=True)
        
        return gaps

    def create_gap_markers(self, gaps, scan_msg):
        """
        Cria marcadores visuais para os gaps detectados.
        
        Args:
            gaps: Lista de gaps detectados
            scan_msg: Mensagem do laser scan original
            
        Returns:
            MarkerArray: Conjunto de marcadores para visualização
        """
        marker_array = MarkerArray()
        lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # Tempo de vida dos marcadores
        
        # Cores para diferentes tipos de gaps baseado na largura
        def get_gap_color(width):
            if width > self.min_gap_width * 2:
                return (0.0, 1.0, 0.0)  # Verde para gaps grandes
            elif width > self.min_gap_width * 1.5:
                return (1.0, 1.0, 0.0)  # Amarelo para gaps médios
            else:
                return (1.0, 0.5, 0.0)  # Laranja para gaps pequenos

        for i, gap in enumerate(gaps):
            # 1. Seta principal indicando direção do gap
            arrow = self._create_base_marker(scan_msg, "gaps_arrows", i, lifetime)
            arrow.type = Marker.ARROW
            
            # Pontos da seta - origem até direção do gap
            start_point = Point(x=0.0, y=0.0, z=0.0)
            distance = min(2.0 * gap['confidence'], self.max_range)
            end_point = Point(
                x=distance * np.cos(gap['angle']),
                y=distance * np.sin(gap['angle']),
                z=0.0
            )
            arrow.points = [start_point, end_point]
            
            # Escala e cor da seta
            arrow.scale.x = 0.1  # Largura da haste
            arrow.scale.y = 0.2  # Largura da ponta
            arrow.scale.z = 0.2
            
            color = get_gap_color(gap['width'])
            arrow.color = ColorRGBA(
                r=color[0], g=color[1], b=color[2], a=gap['confidence']
            )
            marker_array.markers.append(arrow)
            
            # 2. Esfera no ponto médio
            sphere = self._create_base_marker(scan_msg, "gap_centers", i + 1000, lifetime)
            sphere.type = Marker.SPHERE
            sphere.pose.position = Point(
                x=gap['mid_point'][0],
                y=gap['mid_point'][1],
                z=0.0
            )
            
            # Escala proporcional à largura do gap
            sphere_size = min(0.3, gap['width'] / 2)
            sphere.scale.x = sphere_size
            sphere.scale.y = sphere_size
            sphere.scale.z = sphere_size
            
            sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=gap['confidence'])
            marker_array.markers.append(sphere)
            
            # 3. Texto com informações do gap
            text = self._create_base_marker(scan_msg, "gap_info", i + 2000, lifetime)
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position = Point(
                x=gap['mid_point'][0],
                y=gap['mid_point'][1],
                z=0.3  # Texto acima da esfera
            )
            
            text.text = f"W: {gap['width']:.2f}m\nC: {gap['confidence']:.2f}"
            text.scale.z = 0.2  # Tamanho da fonte
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker_array.markers.append(text)
            
            # 4. Linha conectando os pontos do gap (opcional, se quiser visualizar a borda)
            if gap['width'] > self.min_gap_width * 1.5:  # Só para gaps significativos
                line = self._create_base_marker(scan_msg, "gap_lines", i + 3000, lifetime)
                line.type = Marker.LINE_STRIP
                line.scale.x = 0.05  # Espessura da linha
                
                # Pontos da linha
                p1 = Point(
                    x=gap['mid_point'][0] - gap['width']/2 * np.cos(gap['angle'] + np.pi/2),
                    y=gap['mid_point'][1] - gap['width']/2 * np.sin(gap['angle'] + np.pi/2),
                    z=0.0
                )
                p2 = Point(
                    x=gap['mid_point'][0] + gap['width']/2 * np.cos(gap['angle'] + np.pi/2),
                    y=gap['mid_point'][1] + gap['width']/2 * np.sin(gap['angle'] + np.pi/2),
                    z=0.0
                )
                line.points = [p1, p2]
                
                line.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)
                marker_array.markers.append(line)
        
        return marker_array
        
    def _create_base_marker(self, scan_msg, ns, id, lifetime):
        """
        Cria um marcador base com configurações comuns.
        
        Args:
            scan_msg: Mensagem do laser scan original
            ns: Namespace do marcador
            id: ID do marcador
            lifetime: Tempo de vida do marcador
            
        Returns:
            Marker: Marcador base configurado
        """
        marker = Marker()
        marker.header = scan_msg.header
        marker.ns = ns
        marker.id = id
        marker.action = Marker.ADD
        marker.lifetime = lifetime
        return marker

    def scan_callback(self, msg):
        # Encontrar gaps nos dados do laser usando o método de descontinuidade
        gaps = self.find_gaps_discontinuity(msg.ranges, msg.angle_min, msg.angle_increment)
        
        # Criar e publicar marcadores para visualização
        if gaps:
            marker_array = self.create_gap_markers(gaps, msg)
            self.marker_pub.publish(marker_array)
            
            # Log do maior gap encontrado
            largest_gap = gaps[0]  # Já está ordenado por tamanho
            self.get_logger().info(
                f'Largest gap width: {largest_gap["width"]:.2f} m, '
                f'angle: {largest_gap["angle"]:.2f} rad, '
                f'confidence: {largest_gap["confidence"]:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()