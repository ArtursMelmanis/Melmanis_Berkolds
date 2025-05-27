#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotPos            # тот же msg, что читает camera_server

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera_server')
        self.pub = self.create_publisher(RobotPos, '/positions', 10)
        self.sub = self.create_subscription(RobotPos,
                                            '/sim_positions',
                                            self.relay_cb, 10)
        self.get_logger().info('Fake camera server started → /positions')

    def relay_cb(self, msg: RobotPos):
        # ровно пересылаем пакет без изменений
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
