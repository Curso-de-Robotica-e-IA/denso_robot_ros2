import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CapturaImagemReferencia(Node):
    def __init__(self):
        # Evitamos chamar a classe de 'Image' para não dar conflito com o tipo da Mensagem
        super().__init__('get_image_reference')

        # Flag para garantir que vamos salvar apenas o primeiro frame válido
        self.capturado = False
        
        self.br = CvBridge()
        
        # Inscrição no tópico da câmera do Gazebo
        self.sub = self.create_subscription(
            Image, 
            '/basic_camera', 
            self.image_callback, 
            10
        )
        
        self.get_logger().info("Aguardando o primeiro frame do Gazebo para salvar a referência...")

    def image_callback(self, msg):
        # Se já capturamos a imagem, ignora os próximos frames enquanto o nó finaliza
        if self.capturado:
            return

        try:
            # 1. Converte a imagem do ROS2 para OpenCV (BGR de 8 bits)
            frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
            
            # Caminho onde a imagem será salva localmente
            caminho_arquivo = "ref_img.jpeg"
            
            # 2. Salva a imagem (Correção: primeiro o caminho, depois o frame)
            sucesso = cv2.imwrite(caminho_arquivo, frame)
            
            if sucesso:
                self.get_logger().info(f"Sucesso! Imagem de referência salva em: {caminho_arquivo}")
                self.capturado = True
                
                # 3. Força o encerramento controlado do nó após cumprir a tarefa
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            else:
                self.get_logger().error("Falha ao gravar o arquivo em disco. Verifique se a pasta de destino existe.")
                
        except Exception as e:
            self.get_logger().error(f"Erro ao converter a imagem do Gazebo: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CapturaImagemReferencia()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        # Captura a saída limpa do sys.exit(0) quando o nó finaliza sozinho
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # Garante a destruição se o encerramento foi via Ctrl+C antes de capturar
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()