import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor 
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Int8
from cv_bridge import CvBridge
from enum import IntEnum
import cv2
import numpy as np


class ServoStatus(IntEnum):
    INVALID = -1
    OK = 0
    DECELERATE_SINGULARITY = 1
    HALT_SINGULARITY = 2
    DECELERATE_COLLISION = 3
    HALT_COLLISION = 4
    JOINT_BOUND = 5
    DECELERATE_FOR_LEAVING_SINGULARITY = 6


class VisualServoP(Node):
    def __init__(self):
        super().__init__('visual_servo_p')

        self.cb_group = ReentrantCallbackGroup()

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('image_topic', '/basic_camera')
        self.declare_parameter('servo_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('servo_joint_topic', '/servo_node/delta_joint_cmds')
        self.declare_parameter('status_topic', '/servo_node/status')
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        servo_topic = self.get_parameter('servo_topic').get_parameter_value().string_value

        self.robot_status = ServoStatus.OK
        self.kp = 0.0015 
        self.target_u = 320.0 # Center width (640/2)
        self.target_v = 240.0 # Center Height (480/2)
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = "J6"

        self.servo_timer = self.create_timer(0.01, self.control_loop, callback_group=self.cb_group)

        # Subs, Pubs
        self.br = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10, callback_group=self.cb_group)
        self.status_sub = self.create_subscription(Int8, self.get_parameter('status_topic').value, self.status_callback, 10, callback_group=self.cb_group)
        self.joint_pub = self.create_publisher(JointJog, self.get_parameter('servo_joint_topic').value, 10)
        self.twist_pub = self.create_publisher(TwistStamped, servo_topic, 10)
        
        self.get_logger().info("Visual Servo started!")

    
    def status_callback(self, msg):
        self.robot_status = msg.data 

    def image_callback(self, msg):
        # 1. Convert image
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Simple detection (blue)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        moments = cv2.moments(mask)
        
        if moments["m00"] > 100:
            u = moments["m10"] / moments["m00"]
            v = moments["m01"] / moments["m00"]

            error_u = self.target_u - u
            error_v = self.target_v - v

            self.current_twist.twist.linear.y = self.kp * error_u
            self.current_twist.twist.linear.z = self.kp * error_v
            
            cv2.circle(frame, (int(u), int(v)), 10, (0, 255, 0), -1)
            cv2.circle(frame, (int(self.target_u), int(self.target_v)), 10, (0, 255, 255), -1)
            
        else:
            self.current_twist.twist.linear.y = 0.0
            self.current_twist.twist.linear.z = 0.0

        cv2.imshow("Debug Visual Servo", frame)
        cv2.waitKey(1)
    
    def control_loop(self):

        self.get_logger().info("Loop running")
        
        if self.robot_status == ServoStatus.OK:
            self.current_twist.header.stamp = self.system_clock.now().to_msg()
            self.twist_pub.publish(self.current_twist)
            
        elif self.robot_status in [ServoStatus.DECELERATE_SINGULARITY, ServoStatus.HALT_SINGULARITY]:
            self.get_logger().warn(f"Singularity detected ({self.robot_status})! Recovering...")
            
            jog_msg = JointJog()
            jog_msg.header.stamp = self.system_clock.now().to_msg()
            jog_msg.header.frame_id = "J6" 
            jog_msg.joint_names = ['joint_3'] 
            jog_msg.velocities = [-0.1] 
            
            self.joint_pub.publish(jog_msg)
            
        elif self.robot_status == ServoStatus.JOINT_BOUND:
            self.get_logger().error("Joint Limit! Stop the script and move the object.")


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoP()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
