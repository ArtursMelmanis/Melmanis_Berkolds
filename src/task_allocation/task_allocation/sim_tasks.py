#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import math
import random
import time
from robot_msgs.msg import TaskRequest
from robot_msgs.msg import RobotPos

class DummyTasks(Node):
    def __init__(self):
        super().__init__('sim_tasks')
        self.declare_parameter('wave_interval', 30.0)  
        self.declare_parameter('wave_size', 10) 
        self.declare_parameter("zone_names", ["B", "C"])
        self.declare_parameter(
            'zone_centers',  [5.0, 0.0,   -5.0, 0.0]
        )
        self.declare_parameter('spawn_radius', 0.5)
        self.declare_parameter('rand_seed',    -1)

        flat = self.get_parameter('zone_centers').value
        self.zone_centers = [(flat[i], flat[i+1])
                             for i in range(0, len(flat), 2)]
        self.wave_interval = float(self.get_parameter('wave_interval').value)
        self.wave_size = int(self.get_parameter('wave_size').value)
        self.zone_names = list(self.get_parameter('zone_names').value)
        self.r_spawn = float(self.get_parameter('spawn_radius').value)

        seed = int(self.get_parameter('rand_seed').value)
        if seed >= 0:
            import random, numpy as np
            random.seed(seed)
            np.random.seed(seed)
            self.get_logger().info(f"sim_tasks: random seed = {seed}")
        else:
            self.get_logger().info("sim_tasks: random seed = system‑random")

        self.pub = self.create_publisher(TaskRequest, '/task_requests', 10)
        self.wave_timer = self.create_timer(self.wave_interval, self._wave_cb)
        self.first_wave_sent = False
        self.create_subscription(RobotPos,'/sim_positions',self._robot_cb,10)
        
    def _robot_cb(self, msg: RobotPos):
        if not self.first_wave_sent:
            time.sleep(2)
            self._wave_cb()
            self.first_wave_sent = True

    def _wave_cb(self) -> None:
        for _ in range(self.wave_size):
            self.timer_cb()
    
    def timer_cb(self):
        idx_src, idx_dst = random.sample(range(len(self.zone_names)), 2)
        src_name = self.zone_names[idx_src]
        dst_name = self.zone_names[idx_dst]
        sx,  sy  = self.zone_centers[idx_src]
        tx,  ty  = self.zone_centers[idx_dst]
        def rnd(cx, cy):
            theta = random.random()*2*math.pi
            r = random.random()*self.r_spawn
            return cx + r*math.cos(theta), cy + r*math.sin(theta)

        pick_x, pick_y = rnd(sx, sy)
        drop_x, drop_y = rnd(tx, ty)

        msg = TaskRequest()
        msg.pick_x, msg.pick_y = float(pick_x), float(pick_y)
        msg.drop_x, msg.drop_y = float(drop_x), float(drop_y)
        self.pub.publish(msg)

        self.get_logger().info(
            f"New task created: from zone {src_name} to zone {dst_name} "
            f"first coordinates ({pick_x:.2f}, {pick_y:.2f}) "
            f"second coordinates ({drop_x:.2f}, {drop_y:.2f})"
        )

def main():
    rclpy.init()
    node = DummyTasks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
