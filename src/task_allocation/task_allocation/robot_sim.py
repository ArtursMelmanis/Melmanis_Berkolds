#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Dict
from robot_msgs.msg import RobotPos, TaskAssignment, TaskFinished, TaskPenalty, TaskRequest
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
import math, random

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_sim')

        # ── Параметры
        self.declare_parameter('robot_speed', 1.0)
        self.declare_parameter('idle_speed', 1.0)
        self.declare_parameter('drain_rate', 1.0)
        self.declare_parameter('charge_rate', 5.0)
        self.declare_parameter('dead_timeout', 10.0)
        self.declare_parameter('idle_timeout', 5.0)
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('collision_radius', 0.2)
        self.declare_parameter('area_A_center', [0.0, 0.0])
        self.declare_parameter('area_A_radius', 2.0)

        self.speed       = self.get_parameter('robot_speed').value
        self.idle_speed  = self.get_parameter('idle_speed').value
        self.drain_rate  = self.get_parameter('drain_rate').value
        self.charge_rate = self.get_parameter('charge_rate').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        self.dead_timeout = self.get_parameter('dead_timeout').value
        self.N           = self.get_parameter('num_robots').value
        self.CR          = self.get_parameter('collision_radius').value
        self.cx, self.cy = self.get_parameter('area_A_center').value
        self.R           = self.get_parameter('area_A_radius').value       
        
        #self.batteries: Dict[int, float] = {}

        # ── Состояние роботов: для каждого rid храним last_pos и (опционально) маршрут ──
        self.robots = {}
        for rid in range(self.N):
            theta = random.random() * 2 * math.pi
            r = random.random() * self.R
            x = self.cx + r * math.cos(theta)
            y = self.cy + r * math.sin(theta)
            self.robots[rid] = {
                'idle_target': None,
                'last_pos': (x, y),
                'points': None,   # нет активного маршрута
                'progress': 1.0,
                'total_len': 0.0,
                'is_charging': False,
                'penalty_sent': False, 
            }
        
        self.batteries = {rid: 100.0 for rid in self.robots}

        # ── Подписка на назначения от APSO ───────────────────────────
        self.create_subscription(TaskAssignment,
                                 '/task_assignments',
                                 self.on_assignment,
                                 10)

        # ── Паблишеры: текущие позы и факт завершения ────────────────
        self.pos_pub  = self.create_publisher(RobotPos,     '/sim_positions', 10)
        self.done_pub = self.create_publisher(TaskFinished, '/task_finished', 10)
        self.penalty_pub = self.create_publisher(TaskPenalty,  '/task_penalty',  10)
        self.task_req_pub = self.create_publisher(TaskRequest, '/task_requests', 10)
        self.path_clear_pub = self.create_publisher(Int32, '/path_clear', 10)
        
        self.batt_pubs = {}
        for rid in self.robots:
            topic = f'/robot{rid}/battery'
            self.batt_pubs[rid] = self.create_publisher(
                Float32, topic, 10)

        # ── Таймер симуляции (20 Hz) ──────────────────────────────────
        self.create_timer(0.05, self.animate_cb)
        
        # Объявляем параметры для барьера
        self.declare_parameter('blocked_center', [5.0, 5.0])
        self.declare_parameter('blocked_radius', 1.0)
        self.declare_parameter('block_period', 15.0)
        self.declare_parameter('block_duration', 8.0)
        self.declare_parameter('slowdown_factor', 0.1)
        self.blocked_center = np.array(
            self.get_parameter('blocked_center').value, dtype=float
        )
        self.blocked_radius  = float(
            self.get_parameter('blocked_radius').value
        )
        self.block_period    = float(
            self.get_parameter('block_period').value
        )
        self.block_duration  = float(
            self.get_parameter('block_duration').value
        )
        self.slowdown_factor = float(
            self.get_parameter('slowdown_factor').value
        )
        
        self.start_time = self.get_clock().now()
        
        
    def _publish_battery(self, rid):
        msg = Float32()
        msg.data = self.batteries[rid]          # 0–100
        self.batt_pubs[rid].publish(msg)

    def on_assignment(self, msg: TaskAssignment) -> None:
        """Запустить симуляцию маршрута для каждого назначенного робота."""
        for i, rid in enumerate(msg.robot_id):
            # extract pick, drop, origin
            xs = msg.target_x[3*i:3*i+3]
            ys = msg.target_y[3*i:3*i+3]
            # старт = предыдущая last_pos
            x0, y0 = self.robots[rid]['last_pos']
            pts = [
                (x0, y0),
                (xs[0], ys[0]),
                (xs[1], ys[1]),
                (xs[2], ys[2]),
            ]
            # compute total length
            segs = [
                math.hypot(x2-x1, y2-y1)
                for (x1,y1),(x2,y2) in zip(pts[:-1], pts[1:])
            ]
            total = sum(segs) or 1.0

            # сохраняем маршрут
            self.robots[rid].update({
                'points':    pts,
                'progress':  0.0,
                'total_len': total,
                'task_id':   msg.task_id[i],
                'task_coords': (xs[0], ys[0], xs[1], ys[1]),
                'is_charging': False,
                'last_task_time': (
                    self.get_clock().now() - self.start_time
                ).nanoseconds * 1e-9
            })


    def animate_cb(self) -> None:
        """Каждые 0.05 с обновляем позиции всех роботов и публикуем их."""
        msg = RobotPos()
        
        now  = self.get_clock().now().to_msg()
        dt = 0.05
        elapsed = self.get_clock().now() - self.start_time
        sim_time = elapsed.nanoseconds * 1e-9
        
        speeds: Dict[int, float] = {}
        self.current_speed = {}
        for rid, job in list(self.robots.items()):
            x, y = job['last_pos']
                          
            # base speed
            if job['points'] is None:
                speed = self.idle_speed
            else:
                speed = self.speed
                
            if 'is_charging' not in job:
                job['is_charging'] = False
            if 'dead_since' not in job:
                job['dead_since'] = None 
            if 'penalty_sent' not in job:
                job['penalty_sent'] = False
            if rid not in self.batteries:
                self.batteries[rid] = 100.0
            
            dx = x - self.cx
            dy = y - self.cy
            in_zone_A = (dx*dx + dy*dy) <= self.R**2
            
            job.setdefault('last_task_time', sim_time)
            idle_too_long = (job['points'] is None
                     and (sim_time - job['last_task_time'] > self.idle_timeout))
                      
            if idle_too_long and in_zone_A:
                job['is_charging'] = True
                speed = 0.0
                self.batteries[rid] = min(self.batteries[rid] + self.charge_rate*dt, 100.0)
                       
            if job['is_charging']:
                self.batteries[rid] = min(
                    100.0,
                    self.batteries[rid] + self.charge_rate * dt
                )
                speeds[rid] = 0.0
                job['idle_target'] = None
                if self.batteries[rid] >= 100.0:
                    job['is_charging'] = False
                    job['penalty_sent'] = False
                continue
            
            self.batteries[rid] = max(0.0, self.batteries[rid] - self.drain_rate * dt)
            
            if (self.batteries[rid] <= 0.0
                    and not in_zone_A
                    and job['points'] is not None
                    and not job.get('penalty_sent', False)):
                if job['dead_since'] is None:
                    job['dead_since'] = sim_time
                    pen = TaskPenalty()
                    pen.robot_id = rid
                    pen.task_id  = job.get('task_id', -1)
                    pen.stamp    = now
                    self.penalty_pub.publish(pen)
                    job['penalty_sent'] = True
                    path_msg = Int32()
                    path_msg.data = rid
                    self.path_clear_pub.publish(path_msg)
                    if 'task_coords' in job and job['task_coords'] is not None:
                        px, py, dx, dy = job['task_coords']
                        tr = TaskRequest()
                        tr.pick_x, tr.pick_y, tr.drop_x, tr.drop_y = px, py, dx, dy
                        self.task_req_pub.publish(tr)
            if job['dead_since'] is not None:
                if sim_time - job['dead_since'] < self.dead_timeout:
                    speeds[rid] = 0.0
                    continue
                else:
                    # спустя 10 с – переносим в зону A и сбрасываем задание
                    phi = random.random() * 2 * math.pi
                    r_tele = self.R * math.sqrt(random.random())
                    tx = self.cx + r_tele * math.cos(phi)
                    ty = self.cy + r_tele * math.sin(phi)
                    job['last_pos'] = (tx, ty)
                    job['points']       = None
                    job['idle_target']  = None
                    
                    job['waiting_tp']   = False
                    job['is_charging'] = True
                    
                    job['progress']     = 0.0
                    job['total_len']    = 0.0
                    job['dead_since']   = None
                    
                    speeds[rid]         = 0.0
                    self.batteries[rid] = 0.0 
                    continue
            
                
            scaled = speed * (self.batteries[rid] / 100.0)
            speed = scaled if scaled > self.idle_speed else self.idle_speed
                
            # Если барьер активен и робот внутри зоны — замедляем
            t = sim_time % self.block_period
            if t < self.block_duration:
                if np.linalg.norm(np.array([x, y]) - self.blocked_center) < self.blocked_radius:
                    speed *= self.slowdown_factor

            if job['points'] is None:
                # если нет цели — выбираем новую точку в зоне A
                if job['idle_target'] is None:
                    phi = random.random() * 2 *math.pi
                    r = random.random() * self.R
                    tx = self.cx + r*math.cos(phi)
                    ty = self.cy + r*math.sin(phi)
                    job['idle_target'] = (tx, ty)

                # двигаемся к idle_target
                x0, y0 = job['last_pos']
                tx, ty = job['idle_target']
                dx, dy = tx - x0, ty - y0
                dist = math.hypot(dx, dy)
                #step = self.idle_speed * dt
                step = speed * dt
                
                if dist <= step:
                    x, y = tx, ty
                    job['idle_target'] = None   # достигли цели — сбросить
                else:
                    x = x0 + step * dx/dist
                    y = y0 + step * dy/dist
                    
                job['last_pos'] = (x, y)
                
            else:
                s = job['progress'] + speed * dt / job['total_len']
                if s >= 1.0:
                    # маршрут завершён
                    tf = TaskFinished()
                    tf.robot_id = rid
                    tf.stamp    = now
                    self.done_pub.publish(tf)

                    x, y = job['points'][-1]
                    # очистить маршрут
                    job['points']   = None
                    job['progress'] = 1.0
                    job['total_len']= 0.0
                    job['last_pos'] = (x, y)
                    job['last_task_time'] = sim_time
                else:
                    x, y = self._interp(job['points'], s)
                    job['progress'] = s
                    job['last_pos'] = (x, y)
                    
            speeds[rid] = speed
            self.current_speed[rid] = speed

        self.resolve_collisions()
        
        for rid, job in self.robots.items():
            x, y = job['last_pos']
            odom = Odometry()
            odom.header.stamp = now
            odom.child_frame_id = str(rid)
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.twist.twist.linear.x = float(speeds[rid])
            odom.twist.twist.linear.z = float(self.batteries[rid])
            charging_flag = 1.0 if (job['is_charging'] or job.get('waiting_tp', False)) else 0.0
            odom.twist.twist.angular.x = charging_flag
            msg.robot_pos.append(odom)
                   
        self.pos_pub.publish(msg)
        for rid in self.robots:
            self._publish_battery(rid)

    def resolve_collisions(self):
        """
        Раздвигает пары роботов, если они сблизились ближе,
        чем 2 * self.CR.
        """
        # получаем список (rid, job) чтобы не менять self.robots во время итерации
        items = list(self.robots.items())
        for i in range(len(items)):
            rid1, job1 = items[i]
            pos1 = np.array(job1['last_pos'])
            for j in range(i+1, len(items)):
                rid2, job2 = items[j]
                pos2 = np.array(job2['last_pos'])

                diff = pos1 - pos2
                dist = np.linalg.norm(diff)
                min_dist = 2 * self.CR

                if 0 < dist < min_dist:
                    # сколько «перевыталкивать» каждого
                    overlap = (min_dist - dist) * 0.5
                    direction = diff / dist

                    new_pos1 = pos1 + direction * overlap
                    new_pos2 = pos2 - direction * overlap

                    # сохраняем обновлённые позиции обратно в self.robots
                    self.robots[rid1]['last_pos'] = (new_pos1[0], new_pos1[1])
                    self.robots[rid2]['last_pos'] = (new_pos2[0], new_pos2[1])


    @staticmethod
    def _interp(points, t):
        """Линейная интерполяция вдоль ломаной."""
        segs = [
            math.hypot(x2-x1, y2-y1)
            for (x1,y1),(x2,y2) in zip(points[:-1], points[1:])
        ]
        total = sum(segs) or 1.0
        dist  = t * total

        for (x1,y1),(x2,y2),L in zip(points[:-1], points[1:], segs):
            if dist > L:
                dist -= L
            else:
                ratio = dist / L if L else 0.0
                return x1 + ratio*(x2-x1), y1 + ratio*(y2-y1)
        return points[-1]

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
