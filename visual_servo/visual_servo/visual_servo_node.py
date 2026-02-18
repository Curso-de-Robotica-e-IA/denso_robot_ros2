import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor 
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
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

        self.declare_parameter('kp_linear', 0.006)
        self.declare_parameter('kp_angular', 0.007)
        self.declare_parameter('target_u', 320.0)
        self.declare_parameter('target_v', 240.0)

        self.robot_status = ServoStatus.OK
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = "J6"
        
        # Subs, Pubs
        self.br = CvBridge()
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.sub = self.create_subscription(Image, '/basic_camera', self.image_callback, 10, callback_group=self.cb_group)
        self.status_sub = self.create_subscription(Int8, '/servo_node/status', self.status_callback, 10, callback_group=self.cb_group)
        
        self.servo_timer = self.create_timer(0.01, self.control_loop, callback_group=self.cb_group)

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

            error_u = self.get_parameter('target_u').value - u
            error_v = self.get_parameter('target_v').value - v

            kpl = self.get_parameter('kp_linear').value
            kpa = self.get_parameter('kp_angular').value

            self.current_twist.twist.angular.z = kpa * error_u 
            self.current_twist.twist.angular.y = -kpa * error_v
            self.current_twist.twist.linear.y = kpl * error_u
            self.current_twist.twist.linear.z = kpl * error_v
            
            cv2.circle(frame, (int(u), int(v)), 10, (0, 255, 0), -1)
            cv2.circle(frame, (int(self.get_parameter('target_u').value ), int(self.get_parameter('target_v').value)), 10, (0, 255, 255), -1)
            
        else:
            self.reset_twist()

        cv2.imshow("Debug Visual Servo", frame)
        cv2.waitKey(1)
    
    def reset_twist(self):
        self.current_twist.twist.linear.x = 0.0
        self.current_twist.twist.linear.y = 0.0
        self.current_twist.twist.linear.z = 0.0
        self.current_twist.twist.angular.x = 0.0
        self.current_twist.twist.angular.y = 0.0
        self.current_twist.twist.angular.z = 0.0
    
    def control_loop(self):
        self.current_twist.header.stamp = self.system_clock.now().to_msg()
        self.twist_pub.publish(self.current_twist)
            
        if self.robot_status == ServoStatus.JOINT_BOUND:
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
