#!/usr/bin/env python3
"""Publish a conservative virtual touch signal from tool/screen geometry."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def within_touch_region(x, y, z, width, height, gap):
    return abs(x) <= width * 0.47 and abs(y) <= height * 0.47 and 0.0 <= z <= gap


class SimTouchContact(Node):
    def __init__(self):
        super().__init__('sim_touch_contact')
        self.tip = self.declare_parameter('tip_frame', 'left_calib_link').value
        self.screen = self.declare_parameter(
            'screen_frame', 'right_virtual_phone_screen_frame').value
        self.width = self.declare_parameter('phone_width_m', 0.070).value
        self.height = self.declare_parameter('phone_height_m', 0.150).value
        self.gap = self.declare_parameter('contact_gap_m', 0.002).value
        if not all(math.isfinite(x) and x > 0 for x in (self.width, self.height, self.gap)):
            raise ValueError('Phone dimensions and contact gap must be positive and finite')
        if self.gap > 0.01:
            raise ValueError('Virtual contact gap cannot exceed 10 mm')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Bool, '/touch_detected', 10)
        self.create_timer(0.02, self.sample)

    def sample(self):
        contact = False
        try:
            p = self.buffer.lookup_transform(self.screen, self.tip, rclpy.time.Time()).transform.translation
            contact = within_touch_region(
                p.x, p.y, p.z, self.width, self.height, self.gap)
        except TransformException:
            pass  # Missing virtual phone or TF means no contact, never a false positive.
        self.publisher.publish(Bool(data=contact))


def main():
    rclpy.init()
    node = SimTouchContact()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
