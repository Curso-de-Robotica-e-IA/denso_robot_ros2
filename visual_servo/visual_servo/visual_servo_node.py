import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualServoP(Node):
    def __init__(self):
        super().__init__('visual_servo_p')

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('image_topic', '/basic_camera')
        self.declare_parameter('servo_topic', '/servo_node/delta_twist_cmds')
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        servo_topic = self.get_parameter('servo_topic').get_parameter_value().string_value

        self.kp = 0.0015 
        self.target_u = 320.0 # Center width (640/2)
        self.target_v = 240.0 # Center Height (480/2)

        # Subs, Pubs
        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(TwistStamped, servo_topic, 10)
        
        self.get_logger().info("Visual Servo Proporcional iniciado!")

    def image_callback(self, msg):
        # 1. Convert image
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Simple detection (blue)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        moments = cv2.moments(mask)
        
        twist = TwistStamped()
        twist.header.stamp = self.system_clock.now().to_msg()
        twist.header.frame_id = "J6"

        if moments["m00"] > 100:
            u = moments["m10"] / moments["m00"]
            v = moments["m01"] / moments["m00"]

            error_u = self.target_u - u
            error_v = self.target_v - v

            twist.twist.linear.y = self.kp * error_u
            twist.twist.linear.z = self.kp * error_v
            
            cv2.circle(frame, (int(u), int(v)), 10, (0, 255, 0), -1)
        else:
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0

        self.pub.publish(twist)
        cv2.imshow("Debug Visual Servo", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()