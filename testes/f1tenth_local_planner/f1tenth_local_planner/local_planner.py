import numpy as np
from numpy import linalg as LA
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, TransformStamped
import tf2_geometry_msgs

class RRTNode:
    """Classe que representa um nó na árvore RRT."""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.path_cost = float('inf')
        
class LocalPlanner(Node):
    """Implementação do planejador local baseado em RRT para F1TENTH."""
    def __init__(self):
        super().__init__('local_planner')
        
        # Parâmetros da grade de ocupação
        self.grid_size = 20.0  # tamanho total da grade em metros
        self.resolution = 0.05  # metros por célula
        self.grid_dims = int(self.grid_size / self.resolution)
        self.grid = np.zeros((self.grid_dims, self.grid_dims))
        
        # Parâmetros do RRT
        self.max_samples = 200  # número máximo de amostras
        self.step_size = 0.5    # tamanho do passo para expansão
        self.goal_sample_rate = 0.2  # probabilidade de amostrar o objetivo
        self.search_radius = 0.5     # raio de busca para vizinhos
        
        # Estado do robô e planejamento
        self.current_pose = None
        self.goal_pose = None
        self.current_path = []
        self.tree = []
        
        # Parâmetros de controle
        self.max_speed = 2.0  # m/s
        self.max_steer = 0.4  # rad
        self.lookahead = 1.0  # metros
        
        # Publicadores para visualização
        self.tree_pub = self.create_publisher(MarkerArray, '/rrt/tree', 10)
        self.path_pub = self.create_publisher(Marker, '/rrt/path', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/rrt/grid', 10)
        
        # Publicador para comandos de movimento
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 1)
        
        # Timer para o loop principal de planejamento
        self.create_timer(0.1, self.planning_loop)
        self.get_logger().info('Local Planner Initialized')

        # Broadcaster de transformações
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variáveis para os frames
        self.map_frame = 'map'
        self.base_frame = 'ego_racecar'
        self.laser_frame = 'ego_racecar/laser'
        
    def scan_callback(self, scan_msg):
        """Processa dados do LIDAR e atualiza a grade de ocupação."""
        self.get_logger().info(f'Recebido scan com {len(scan_msg.ranges)} pontos')
        self.get_logger().info(f'Frame do scan: {scan_msg.header.frame_id}')
        if self.current_pose is None:
            self.get_logger().warn('Pose atual não disponível ainda')
            return
            
        # Atualiza a grade de ocupação
        self.update_occupancy_grid(scan_msg)
        
        # Publica a grade para visualização
        self.publish_grid()
        
    def pose_callback(self, odom_msg):
        """Atualiza a pose atual do robô."""
        self.current_pose = odom_msg.pose.pose
        # Log detalhado apenas na primeira mensagem recebida
        if not hasattr(self, '_received_first_odom'):
            self.get_logger().info('Primeira mensagem de odometria recebida!')
            self.get_logger().info(f'Frame ID: {odom_msg.header.frame_id}')
            self._received_first_odom = True
            
        self.get_logger().debug('Pose atualizada: x=%.2f, y=%.2f' % 
                              (self.current_pose.position.x, self.current_pose.position.y))
        # Publica transformação map -> ego_racecar
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.current_pose.position.x
        t.transform.translation.y = self.current_pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.current_pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publica transformação ego_racecar -> ego_racecar/laser
        # Ajuste estes valores de acordo com a posição real do seu LIDAR
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = self.base_frame
        t_laser.child_frame_id = self.laser_frame
        t_laser.transform.translation.x = 0.0  # Ajuste conforme a posição do LIDAR
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.0
        t_laser.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t_laser)
        
    def goal_callback(self, goal_msg):
        """Atualiza o objetivo atual."""
        self.goal_pose = goal_msg.pose
        self.current_path = []  # Limpa o caminho atual
        self.get_logger().info('Novo objetivo recebido: x=%.2f, y=%.2f' % 
                              (goal_msg.pose.position.x, goal_msg.pose.position.y))
        
    def update_occupancy_grid(self, scan_msg):
        """Atualiza a grade de ocupação local usando dados do LIDAR."""
        # Limpa a grade
        self.grid.fill(0)
        
        # Converte varreduras para coordenadas cartesianas
        angles = np.arange(
            scan_msg.angle_min,
            scan_msg.angle_max + scan_msg.angle_increment,
            scan_msg.angle_increment
        )
        
        # Filtra leituras inválidas
        ranges = np.array(scan_msg.ranges)
        valid = ranges < scan_msg.range_max
        
        # Calcula pontos x,y
        x = np.cos(angles[valid]) * ranges[valid]
        y = np.sin(angles[valid]) * ranges[valid]
        
        # Converte para índices da grade
        grid_x = ((x + self.grid_size/2) / self.resolution).astype(int)
        grid_y = ((y + self.grid_size/2) / self.resolution).astype(int)
        
        # Marca células ocupadas
        valid_cells = (grid_x >= 0) & (grid_x < self.grid_dims) & \
                     (grid_y >= 0) & (grid_y < self.grid_dims)
        self.grid[grid_y[valid_cells], grid_x[valid_cells]] = 100
        
        # Aplica inflação para segurança
        self.inflate_obstacles()
        
    def inflate_obstacles(self, radius=0.2):
        """Infla obstáculos na grade para criar margem de segurança."""
        kernel_size = int(radius / self.resolution)
        y, x = np.ogrid[-kernel_size:kernel_size+1, -kernel_size:kernel_size+1]
        kernel = x*x + y*y <= kernel_size*kernel_size
        
        # Aplica dilatação
        occupied = self.grid > 50
        self.grid = np.logical_or(occupied, 
            np.convolve(occupied.astype(float), 
                       kernel.astype(float), 
                       mode='same') > 0).astype(int) * 100
                       
    def sample_free_space(self):
        """Gera amostra no espaço livre com bias para frente do carro."""
        if self.current_pose is None or self.goal_pose is None:
            return None
            
        if np.random.random() < self.goal_sample_rate:
            # Amostra próxima ao objetivo
            return self.goal_pose.position.x, self.goal_pose.position.y
            
        while True:
            # 80% das amostras em um cone à frente
            if np.random.random() < 0.8:
                # Calcula ângulo do carro
                yaw = 2 * math.atan2(self.current_pose.orientation.z,
                                   self.current_pose.orientation.w)
                                   
                # Amostra no cone
                angle = yaw + np.random.uniform(-math.pi/4, math.pi/4)
                r = np.random.uniform(0, self.grid_size/2)
                x = self.current_pose.position.x + r * math.cos(angle)
                y = self.current_pose.position.y + r * math.sin(angle)
            else:
                # 20% em todo espaço
                x = np.random.uniform(-self.grid_size/2, self.grid_size/2)
                y = np.random.uniform(-self.grid_size/2, self.grid_size/2)
                
            # Verifica se está livre
            if not self.is_collision(x, y):
                return x, y
                
    def nearest_node(self, point):
        """Encontra o nó mais próximo na árvore."""
        min_dist = float('inf')
        nearest = None
        
        for node in self.tree:
            dist = math.hypot(node.x - point[0], node.y - point[1])
            if dist < min_dist:
                min_dist = dist
                nearest = node
                
        return nearest
        
    def steer(self, from_node, to_point):
        """Cria novo nó na direção do ponto mantendo distância máxima."""
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dist = math.hypot(dx, dy)
        
        if dist <= self.step_size:
            new_x = to_point[0]
            new_y = to_point[1]
        else:
            theta = math.atan2(dy, dx)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            
        return RRTNode(new_x, new_y)
        
    def is_collision(self, x, y):
        """Verifica se um ponto está em colisão na grade."""
        # Converte para índices da grade
        grid_x = int((x + self.grid_size/2) / self.resolution)
        grid_y = int((y + self.grid_size/2) / self.resolution)
        
        # Verifica limites
        if grid_x < 0 or grid_x >= self.grid_dims or \
           grid_y < 0 or grid_y >= self.grid_dims:
            return True
            
        return self.grid[grid_y, grid_x] > 50
        
    def check_path(self, from_node, to_node):
        """Verifica se o caminho entre dois nós está livre."""
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.hypot(dx, dy)
        
        # Verifica pontos ao longo do caminho
        steps = int(dist / (self.resolution/2))
        for i in range(steps):
            t = float(i) / steps
            x = from_node.x + t * dx
            y = from_node.y + t * dy
            if self.is_collision(x, y):
                return False
                
        return True
        
    def planning_loop(self):
        """Loop principal de planejamento."""
        # Usamos apenas warn() em vez de warn_throttle
        if self.current_pose is None:
            self.get_logger().warn('Aguardando pose inicial...')
            return
            
        if self.goal_pose is None:
            self.get_logger().warn('Aguardando objetivo...')
            return
            
        # Loop principal de planejamento
        if not self.current_path:
            self.get_logger().info('Planejando novo caminho...')
            path = self.plan_path()
            if path:
                self.current_path = path
                self.publish_path(path)
                self.get_logger().info('Novo caminho encontrado!')
            else:
                self.get_logger().warn('Não foi possível encontrar um caminho')
                
        # Executa o caminho atual
        if self.current_path:
            self.execute_path()
            self.get_logger().debug('Executando caminho...')
            
        # Se não tem caminho, planeja novo
        if not self.current_path:
            path = self.plan_path()
            if path:
                self.current_path = path
                self.publish_path(path)
                
        # Executa o caminho atual
        if self.current_path:
            self.execute_path()
            
    def plan_path(self):
        """Executa o algoritmo RRT para encontrar um caminho."""
        # Inicializa árvore com posição atual
        self.tree = []
        start_node = RRTNode(self.current_pose.position.x,
                            self.current_pose.position.y)
        start_node.path_cost = 0
        self.tree.append(start_node)
        
        # Criação da árvore
        for _ in range(self.max_samples):
            # Amostra ponto
            point = self.sample_free_space()
            if point is None:
                continue
                
            # Encontra vizinho mais próximo
            nearest = self.nearest_node(point)
            
            # Expande árvore
            new_node = self.steer(nearest, point)
            
            # Verifica colisão
            if self.check_path(nearest, new_node):
                new_node.parent = nearest
                self.tree.append(new_node)
                
                # Verifica se chegou ao objetivo
                dist_to_goal = math.hypot(
                    new_node.x - self.goal_pose.position.x,
                    new_node.y - self.goal_pose.position.y
                )
                if dist_to_goal < self.search_radius:
                    return self.extract_path(new_node)
                    
            # Visualiza árvore
            self.publish_tree()
            
        return None
        
    def extract_path(self, node):
        """Extrai o caminho da árvore até o nó dado."""
        path = []
        current = node
        while current is not None:
            path.append(current)
            current = current.parent
        return path[::-1]
        
    def execute_path(self):
        """Executa o caminho atual usando pure pursuit."""
        if not self.current_path:
            return
            
        # Encontra o ponto de objetivo no caminho
        target_idx = self.find_target_point()
        if target_idx is None:
            return
            
        # Calcula comando de controle
        target = self.current_path[target_idx]
        cmd = self.compute_control(target)
        
        # Publica comando
        self.cmd_pub.publish(cmd)
        
    def find_target_point(self):
        """Encontra ponto alvo no caminho usando lookahead."""
        if not self.current_path:
            return None
            
        # Encontra ponto mais próximo no caminho
        min_idx = 0
        min_dist = float('inf')
        
        for i, node in enumerate(self.current_path):
            dist = math.hypot(
                node.x - self.current_pose.position.x,
                node.y - self.current_pose.position.y
            )
            if dist < min_dist:
                min_dist = dist
                min_idx = i
                
        # Procura ponto de lookahead
        for i in range(min_idx, len(self.current_path)):
            dist = math.hypot(
                self.current_path[i].x - self.current_pose.position.x,
                self.current_path[i].y - self.current_pose.position.y
            )
            if dist >= self.lookahead:
                return i
                
        # Se não encontrar, usa último ponto
        return len(self.current_path) - 1
        
    def compute_control(self, target):
        """Calcula comando de velocidade usando pure pursuit."""
        cmd = Twist()
        
        # Calcula erro de ângulo
        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y
        target_angle = math.atan2(dy, dx)
        
        # Ângulo atual do carro
        yaw = 2 * math.atan2(self.current_pose.orientation.z,
                            self.current_pose.orientation.w)
                            
        # Erro de ângulo
        angle_error = target_angle - yaw
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        # Controle proporcional
        cmd.angular.z = max(-self.max_steer,
                          min(self.max_steer, 1.5 * angle_error))
                          
        # Velocidade proporcional ao erro de ângulo
        cmd.linear.x = self.max_speed * (1 - abs(angle_error) / math.pi)
        return cmd
        
    def publish_tree(self):
        """Publica a árvore RRT para visualização no RViz."""
        marker_array = MarkerArray()
        
        # Marcador para nós
        node_marker = Marker()
        node_marker.header.frame_id = self.map_frame
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.1
        node_marker.scale.y = 0.1
        node_marker.color.r = 0.0
        node_marker.color.g = 1.0
        node_marker.color.b = 0.0
        node_marker.color.a = 1.0
        
        # Marcador para arestas
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.02
        edge_marker.color.r = 0.0
        edge_marker.color.g = 0.0
        edge_marker.color.b = 1.0
        edge_marker.color.a = 0.5
        
        # Adiciona nós e arestas
        for node in self.tree:
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            node_marker.points.append(p)
            
            if node.parent is not None:
                p1 = Point()
                p1.x = node.x
                p1.y = node.y
                p1.z = 0.0
                edge_marker.points.append(p1)
                
                p2 = Point()
                p2.x = node.parent.x
                p2.y = node.parent.y
                p2.z = 0.0
                edge_marker.points.append(p2)
                
        marker_array.markers.append(node_marker)
        marker_array.markers.append(edge_marker)
        
        # Publica marcadores
        self.tree_pub.publish(marker_array)
        
    def publish_path(self, path):
        """Publica o caminho planejado para visualização."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Adiciona pontos do caminho
        for node in path:
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            marker.points.append(p)
            
        self.path_pub.publish(marker)
        
    def publish_grid(self):
        """Publica a grade de ocupação para visualização."""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = self.map_frame
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_dims
        grid_msg.info.height = self.grid_dims
        grid_msg.info.origin.position.x = -self.grid_size/2
        grid_msg.info.origin.position.y = -self.grid_size/2
        
        # Converte grade para mensagem
        grid_msg.data = self.grid.flatten().astype(int).tolist()
        
        self.grid_pub.publish(grid_msg)

def main(args=None):
    """Função principal para inicializar o nó ROS."""
    rclpy.init(args=args)
    node = LocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()