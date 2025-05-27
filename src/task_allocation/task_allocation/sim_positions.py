#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math, random
from robot_msgs.msg import RobotPos
from nav_msgs.msg import Odometry

class DummyPositions(Node):
    def __init__(self):
        super().__init__('sim_positions')
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('area_A_center', [0.0, 0.0])
        self.declare_parameter('area_A_radius', 1.0)
        self.N = self.get_parameter('num_robots').value
        self.cx, self.cy = self.get_parameter('area_A_center').value
        self.R = self.get_parameter('area_A_radius').value

        self.pub = self.create_publisher(RobotPos, '/sim_positions', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = RobotPos()
        for i in range(self.N):
            θ = random.random() * 2*math.pi
            r = random.random() * self.R
            x = self.cx + r * math.cos(θ)
            y = self.cy + r * math.sin(θ)
            odom = Odometry()
            odom.child_frame_id = str(i)
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            msg.robot_pos.append(odom)
        self.pub.publish(msg)
        self.get_logger().debug(f'Published robot positions in A')

def main():
    rclpy.init()
    node = DummyPositions()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

