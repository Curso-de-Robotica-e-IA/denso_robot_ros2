import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from cv_bridge import CvBridge
from enum import IntEnum
import cv2
import numpy as np
import torch
from pathlib import Path

# --- Imports do seu Pipeline de Visão ---
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd

# Certifique-se de importar suas funções estruturadas do seu arquivo de validação
from src.validation_pipeline import (
    match_histogram_tensor,
    apply_bilateral_filter,
    apply_canny_edge,
    run_boruvka,
    estimate_homography
)

class ServoStatus(IntEnum):
    INVALID = -1
    OK = 0
    DECELERATE_SINGULARITY = 1
    HALT_SINGULARITY = 2
    DECELERATE_COLLISION = 3
    HALT_COLLISION = 4
    JOINT_BOUND = 5
    DECELERATE_FOR_LEAVING_SINGULARITY = 6


class VisualServoHomografia(Node):
    def __init__(self):
        super().__init__('visual_servo_homografia')
        self.cb_group = ReentrantCallbackGroup()
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # --- Parâmetros de Controle (Ajuste fino no robô) ---
        self.declare_parameter('kp_linear', 0.0005)   # Menor ganho por trabalhar com erro acumulado de 4 cantos
        self.declare_parameter('kp_angular', 0.0008)
        self.declare_parameter('erro_parada_pixels', 3.0) # Limiar de erro médio dos cantos para parar o robô
        
        # --- Inicialização dos Modelos de Deep Learning na GPU ---
        torch.set_grad_enabled(False)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Modelos de Visão alocados em: {self.device}")
        
        self.extractor = SuperPoint(max_num_keypoints=1024).eval().to(self.device)
        self.matcher = LightGlue(feature="superpoint", flash=True).eval().to(self.device)

        # --- Carregamento Prévio e Processamento da Imagem de Referência (Alvo) ---
        path_ref = Path("ref_image.jpeg")
        if not path_ref.exists():
            self.get_logger().error(f"Imagem de referência {path_ref} não encontrada!")
            raise FileNotFoundError()
            
        # Carrega e prepara a Imagem de Referência Estática
        img_raw = cv2.imread(str(path_ref))
        self.h_img, self.w_img, _ = img_raw.shape
        
        # Define as quinas virtuais padrão da tela (Sera nosso Alvo Fixo s*)
        self.cantos_referencia = np.array([
            [0, 0],                  # Sup. Esquerdo
            [self.w_img, 0],         # Sup. Direito
            [self.w_img, self.h_img],# Inf. Direito
            [0, self.h_img]          # Inf. Esquerdo
        ], dtype=np.float32).reshape(-1, 1, 2)
        
        # Converte para Tensor e pré-processa os Superpixels da Imagem de Referência
        self.ref_tensor = torch.from_numpy(img_raw).permute(2, 0, 1).float().to(self.device) / 255.0
        ref_edge = apply_canny_edge(self.ref_tensor)
        ref_bf = apply_bilateral_filter(self.ref_tensor)
        # Deixa a imagem segmentada fixa na memória para não recalcular a cada frame
        self.ref_proc = run_boruvka(ref_bf, edge_map=ref_edge, n_supix=100)
        self.feats_ref = self.extractor.extract(self.ref_proc)

        # --- Variáveis de Estado do Robô ---
        self.robot_status = ServoStatus.OK
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = "J6" # Quadro de coordenadas da ferramenta do DENSO
        
        # Subs, Pubs e Timers
        self.br = CvBridge()
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.sub = self.create_subscription(Image, '/basic_camera', self.image_callback, 10, callback_group=self.cb_group)
        self.status_sub = self.create_subscription(Int8, '/servo_node/status', self.status_callback, 10, callback_group=self.cb_group)
        
        # Loop de envio do comando de velocidade constante para o MoveIt Servo (100Hz)
        self.servo_timer = self.create_timer(0.01, self.control_loop, callback_group=self.cb_group)
        self.get_logger().info("Visual Servo Não-Calibrado via Homografia iniciado!")

    def status_callback(self, msg):
        self.robot_status = msg.data 

    def image_callback(self, msg):
        # 1. Conversão e Tratamento do Frame Atual da Câmera do Robô
        frame_raw = self.br.imgmsg_to_cv2(msg, "bgr8")
        cur_tensor = torch.from_numpy(frame_raw).permute(2, 0, 1).float().to(self.device) / 255.0
        
        # 2. Pipeline de Alinhamento e Filtragem (Borůvka + Filtros)
        cur_hm = match_histogram_tensor(cur_tensor, self.ref_tensor)
        cur_edge = apply_canny_edge(cur_hm)
        cur_bf = apply_bilateral_filter(cur_hm)
        cur_proc = run_boruvka(cur_bf, edge_map=cur_edge, n_supix=100)
        
        # 3. Matching Adaptativo com LightGlue
        feats_cur = self.extractor.extract(cur_proc)
        matches0 = self.matcher({"image0": self.feats_ref, "image1": feats_cur, "filter_threshold": 0.1})
        
        feats_ref_rbd, feats_cur_rbd, matches0 = [rbd(x) for x in [self.feats_ref, feats_cur, matches0]]
        matches = matches0["matches"]
        
        if len(matches) < 4:
            self.get_logger().warn("Poucos matches para calcular a homografia!")
            self.reset_twist()
            return

        kpts_ref = feats_ref_rbd["keypoints"][matches[..., 0]].cpu().numpy()
        kpts_cur = feats_cur_rbd["keypoints"][matches[..., 1]].cpu().numpy()

        # 4. Cálculo da Homografia com Sentido Geométrico Ajustado para Abordagem B
        # Mapeamos da Imagem de Referência (0) para a Imagem Atual (1)
        H, mask, inliers = estimate_homography(kpts_ref, kpts_cur)
        
        if H is not None and inliers >= 4:
            # 5. ABORDAGEM B: Projeta as quinas virtuais da Referência na imagem Atual
            cantos_projetados = cv2.perspectiveTransform(self.cantos_referencia, H)
            cantos_est = cantos_projetados.squeeze()
            cantos_ref_flat = self.cantos_referencia.squeeze()

            # Calcular os erros individuais em X e Y acumulados dos 4 cantos
            # Isso gera as componentes de erro de translação e rotação plana
            erro_x = np.sum(cantos_ref_flat[:, 0] - cantos_est[:, 0])
            erro_y = np.sum(cantos_ref_flat[:, 1] - cantos_est[:, 1])
            
            # Erro médio absoluto em pixels para critério de convergência/parada
            erro_medio = np.mean(np.linalg.norm(cantos_ref_flat - cantos_est, axis=1))

            if erro_medio < self.get_parameter('erro_parada_pixels').value:
                self.get_logger().info(f"Alvo alcançado! Erro médio: {erro_medio:.2f} px. Parando manipulador.")
                self.reset_twist()
            else:
                kpl = self.get_parameter('kp_linear').value
                kpa = self.get_parameter('kp_angular').value

                # Mapeamento Proporcional dos Erros Virtuais para as Velocidades do MoveIt Servo
                # Ajuste a inversão dos eixos (+/-) conforme o comportamento prático da sua câmera no punho J6
                self.current_twist.twist.linear.x = kpl * erro_x
                self.current_twist.twist.linear.y = kpl * erro_y
                self.current_twist.twist.angular.z = kpa * (cantos_est[1, 1] - cantos_ref_flat[1, 1]) # Exemplo simples de torção no eixo Z baseado no desalinhamento Y do canto superior direito
        else:
            self.reset_twist()

        # Janela de visualização em tempo real interna do nó (Mantenha desativada em produção para não pesar)
        cv2.imshow("Visual Servo - Malha Fechada", frame_raw)
        cv2.waitKey(1)

    def reset_twist(self):
        self.current_twist.twist.linear.x = 0.0
        self.current_twist.twist.linear.y = 0.0
        self.current_twist.twist.linear.z = 0.0
        self.current_twist.twist.angular.x = 0.0
        self.current_twist.twist.angular.y = 0.0
        self.current_twist.twist.angular.z = 0.0

    def control_loop(self):
        # Envia a mensagem com timestamp atualizado exigido pelo MoveIt Servo
        self.current_twist.header.stamp = self.system_clock.now().to_msg()
        self.twist_pub.publish(self.current_twist)
            
        if self.robot_status == ServoStatus.JOINT_BOUND:
            self.get_logger().error("MoveIt Servo: Limite de Junta atingido! Travando movimentação por segurança.")
            self.reset_twist()


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoHomografia()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()