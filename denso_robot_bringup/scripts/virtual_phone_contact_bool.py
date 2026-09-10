#!/usr/bin/env python3
"""Publish whether the toucher TCP is currently contacting the virtual glass."""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool


class VirtualPhoneContactBoolean(Node):
    def __init__(self):
        super().__init__('virtual_phone_contact_boolean')
        self.declare_parameter('contact_topic', '/left_calib_contact')
        self.declare_parameter('output_topic', '/left/calib_contact_detected')
        self.declare_parameter('tip_collision', 'left_calib_contact')
        self.declare_parameter('screen_collision', 'right_virtual_phone_screen_collision')
        self.declare_parameter('contact_timeout_s', 0.05)

        contact_topic = self.get_parameter('contact_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.tip_collision = self.get_parameter('tip_collision').value
        self.screen_collision = self.get_parameter('screen_collision').value
        self.timeout_ns = int(self.get_parameter('contact_timeout_s').value * 1e9)
        self.last_contact_ns = None

        self.publisher = self.create_publisher(Bool, output_topic, 10)
        self.subscription = self.create_subscription(Contacts, contact_topic, self.on_contacts, 10)
        self.timer = self.create_timer(0.01, self.publish_state)

    def on_contacts(self, message):
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if (any(self.tip_collision in name for name in names) and
                    any(self.screen_collision in name for name in names)):
                self.last_contact_ns = self.get_clock().now().nanoseconds
                return

    def publish_state(self):
        now_ns = self.get_clock().now().nanoseconds
        active = self.last_contact_ns is not None and now_ns - self.last_contact_ns <= self.timeout_ns
        self.publisher.publish(Bool(data=active))


def main():
    rclpy.init()
    node = VirtualPhoneContactBoolean()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
