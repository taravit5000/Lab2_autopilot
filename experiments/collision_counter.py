#!/usr/bin/env python3
import signal

import rclpy
from gazebo_msgs.msg import ContactsState
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from rclpy.node import Node


class CollisionCounter(Node):
    def __init__(self):
        super().__init__('collision_counter')
        self.count = 0
        self.last_contact = float('-inf')
        self.robot_prefix = self.declare_parameter('robot_name', 'burger').value + '::'
        links = ['base_link', 'base_scan', 'wheel_left_link',
                 'wheel_right_link', 'caster_back_link']
        self.subscriptions_ = [
            self.create_subscription(ContactsState, '/contacts/' + link,
                                     self.on_contact, 100)
            for link in links
        ]
        self.get_logger().info('Collisions: 0')

    def on_contact(self, message):
        external_contact = False
        for state in message.states:
            a, b = state.collision1_name, state.collision2_name
            if 'ground_plane' in a or 'ground_plane' in b:
                continue
            if a.startswith(self.robot_prefix) != b.startswith(self.robot_prefix):
                external_contact = True
                break
        if not external_contact:
            return

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp < self.last_contact - 0.5:
            self.count = 0
            self.last_contact = float('-inf')
        # Merge repeated contact messages into one event.
        if stamp - self.last_contact > 0.2:
            self.count += 1
            self.get_logger().info(f'Collisions: {self.count}')
        self.last_contact = max(self.last_contact, stamp)


def main():
    rclpy.init()
    node = CollisionCounter()
    app = QApplication([])
    window = QWidget()
    window.setWindowTitle('Collision counter')
    window.setWindowFlag(Qt.WindowStaysOnTopHint)
    layout = QVBoxLayout(window)
    label = QLabel('Collisions: 0')
    label.setStyleSheet('font-size: 24px; padding: 12px;')
    layout.addWidget(label)

    def refresh():
        if not rclpy.ok():
            app.quit()
            return
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0)
        label.setText(f'Collisions: {node.count}')

    timer = QTimer()
    timer.timeout.connect(refresh)
    timer.start(20)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
