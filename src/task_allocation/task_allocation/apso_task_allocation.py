#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import os
import sys
import numpy as np
import itertools
import math
import time
from rclpy.node import Node
from typing import Dict, Optional, List
from collections import deque
from concurrent.futures import ThreadPoolExecutor, TimeoutError
from robot_msgs.msg import RobotPos, TaskRequest, TaskAssignment, TaskFinished, TaskPenalty
from std_msgs.msg import Float32
from .apso_core.APSO import APSO

class APSOAllocator(Node):
    def __init__(self):
        super().__init__('apso_task_allocation')
        
        self.declare_parameter('allocation_mode', 'adaptive')
        self.mode = self.get_parameter('allocation_mode').value.lower()
        self.get_logger().info(f"Allocation mode = {self.mode.upper()}")
        
        self.opt_executor = ThreadPoolExecutor(max_workers=1)
        self.TimeoutError = TimeoutError

        self.declare_parameter('max_tasks', 50)
        self.max_tasks = self.get_parameter('max_tasks').value
        self.tasks_received = 0
        self.tasks_assigned = 0

        ws = os.path.expanduser('~/heroswarmv2/ROS2/ros_ws')
        stats_dir = os.path.join(ws, 'src', 'task_allocation', 'apso_stats')
        os.makedirs(stats_dir, exist_ok=True)
        idx_next = max(
            [int(fn[len('statistics_'):-4]) for fn in os.listdir(stats_dir)
             if fn.startswith('statistics_') and fn.endswith('.csv')] or [0]
        ) + 1
        self.stats_file = os.path.join(stats_dir, f'statistics_{idx_next}.csv')
        with open(self.stats_file, 'w', encoding='utf-8') as f:
            f.write('Generated_tasks,Completed_tasks,Uncompleted_tasks,Cost\n')
        self.get_logger().info(f"Statistics file: {self.stats_file}")

        self.declare_parameter('summary_file', 'summary2.csv')
        raw = self.get_parameter('summary_file').value
        self.summary_file = raw if os.path.isabs(raw) else os.path.join(stats_dir, raw)
        self.get_logger().info(f"Summary will be written to: {self.summary_file}")

        self.declare_parameter('pso_population', 50)
        self.declare_parameter('pso_generations', 200)
        self.declare_parameter('optimization_timeout', 0.05)
        self.pso_P = self.get_parameter('pso_population').value
        self.pso_G = self.get_parameter('pso_generations').value
        self.optimization_timeout = self.get_parameter('optimization_timeout').value
        
        self.declare_parameter('batch_size', 5)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('beta', 2.5)
        self.declare_parameter('risk_penalty', 1e4)
        self.declare_parameter('collision_penalty', 1e5)
        self.declare_parameter('soc_reserve', 20.0)
        
        self.batch_size        = self.get_parameter('batch_size').value
        self.alpha             = self.get_parameter('alpha').value
        self.beta              = self.get_parameter('beta').value
        self.risk_penalty      = self.get_parameter('risk_penalty').value
        self.collision_penalty = self.get_parameter('collision_penalty').value
        self.soc_reserve = self.get_parameter('soc_reserve').value
        
        self.declare_parameter('summary_block_size', 5)
        self.summary_block_size = self.get_parameter('summary_block_size').value
        self.number = 0
        
        self.apso   = None
        self.robots = {}
        self.batteries = {}
        self.batt_subs = {}
        self.ids_in_batch: List[int] = []
        self.tasks  = deque(maxlen=100)
        self.pool   = ThreadPoolExecutor(max_workers=1)
        self.busy   = set()
        
        #self.assignment_info: Dict[Tuple[int,int], Tuple[float,float]] = {}
        self.next_task_id = 0
        self.assignment_info = {}
        self.dynamic_ratio = 0.0
        self.failed_thresholds = []
        
        self.robot_charging: Dict[int, bool] = {} 
        self.robots_speeds: Dict[int, float] = {}
        self.declare_parameter('default_speed', 1.0)
        self.default_speed = self.get_parameter('default_speed').value

        self.create_subscription(RobotPos,   '/sim_positions', self.pos_cb, 10)
        self.create_subscription(TaskRequest,'/task_requests',   self.task_cb,10)
        self.assign_pub = self.create_publisher(TaskAssignment,'/task_assignments',10)
        self.create_subscription(
            TaskFinished, '/task_finished', self.finished_cb, 10
        )
        self.create_subscription(TaskPenalty,'/task_penalty',self.penalty_cb,10)
        
        self.wave_id = 0
        self.declare_parameter('penalty_idle_time', 10.0)
        self.penalty_idle_time = self.get_parameter('penalty_idle_time').value
        self.wave_timer = self.create_timer(3.0, self._maybe_start)
        self.task_start_times = {}
        self.task_durations:    List[float] = []
        self.penalties = 0
        self.start_time = self.get_clock().now()

    def _write_stats(self, cost=None):
        unassigned = self.tasks_received - self.tasks_assigned
        cost_str = f"{cost:.2f}" if cost is not None else ''
        with open(self.stats_file, 'a', encoding='utf-8') as f:
            f.write(f"{self.tasks_received},"
                    f"{self.tasks_assigned},"
                    f"{unassigned},"
                    f"{cost_str}\n")

    def _ensure_robot(self, rid: int):
        if rid in self.batteries:
            return

        self.batteries[rid] = 100.0
        topic = f'/robot{rid}/battery'
        self.batt_subs[rid] = self.create_subscription(
            Float32, topic,
            lambda msg, rid=rid: self.batt_cb(rid, msg),
            10)
    
    def batt_cb(self, rid: int, msg: Float32):
        self.batteries[rid] = float(msg.data)
        
    def penalty_cb(self, msg: TaskPenalty):
        self.penalties += 1
        if self.mode == 'static':
            return
        key = (msg.robot_id, msg.task_id)
        if key not in self.assignment_info:
            self.get_logger().warn(f"No assignment info for {key}")
            return
        route_len, soc_start = self.assignment_info.pop(key)
        thr = soc_start / route_len
        self.dynamic_ratio = max(self.dynamic_ratio, thr)
        self.failed_thresholds.append(thr)
        now_ros = self.get_clock().now()
        start_ros = self.task_start_times.pop(msg.task_id, None)
        if start_ros is not None:
            dt = (now_ros - start_ros).nanoseconds * 1e-9
            self.task_durations.append(dt + self.penalty_idle_time)
            self.get_logger().info(f"Task ended at {now_ros.nanoseconds * 1e-9:.3f}s")
        else:
            self.get_logger().warn(f"No start time for penalized tid={msg.task_id}")
        self.get_logger().info(
            f"Penalty for R{msg.robot_id}, T{msg.task_id}: "
            f"route={route_len:.1f}, soc_start={soc_start:.1f}, thr={thr:.3f}"
        )
        self.robot_charging[msg.robot_id] = True

    def pos_cb(self, msg: RobotPos):
        any_new_free = False
        for odom in msg.robot_pos:
            rid = int(odom.child_frame_id)
            self._ensure_robot(rid) 
            
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            self.robots[rid] = (x, y)
            self.robots_speeds[rid] = max(
                float(odom.twist.twist.linear.x), 0.05)
            
            was_charging = self.robot_charging.get(rid, False)
            charging = odom.twist.twist.angular.x > 0.5
            self.robot_charging[rid] = charging
            battery = float(odom.twist.twist.linear.z)
            self.batteries[rid] = battery
            if (was_charging and not charging
                    and battery >= 100.0
                    and rid in self.busy):
                self.busy.remove(rid)
                any_new_free = True
                
            if rid not in self.robots:
                self.get_logger().info(
                    f"Robot {rid} appeared in zone A at position ({x:.2f}, {y:.2f})"
                )
                
        if any_new_free and self.tasks:
            self._maybe_start()

    def task_cb(self, msg: TaskRequest):
        pick_x, pick_y = msg.pick_x, msg.pick_y
        drop_x, drop_y = msg.drop_x, msg.drop_y

        self.tasks.append((pick_x, pick_y, drop_x, drop_y))
        self.tasks_received += 1
        self._write_stats(cost=None)

        self.get_logger().info(
            f"Task received: first coordinates = ({pick_x:.2f},{pick_y:.2f}) "
            f"-> second coordinates = ({drop_x:.2f},{drop_y:.2f})"
        )

    def _maybe_start(self):
        if getattr(self, '_optimizing', False):
            return
        if not self.robots or len(self.tasks) == 0:
            return
            
        if self.mode != 'adaptive':
            self._static_assign()
            return

        batch_tasks = list(itertools.islice(self.tasks, 0, self.batch_size))
        if not batch_tasks:
            return

        free0 = [r for r in self.robots
                 if r not in self.busy
                 and not self.robot_charging.get(r, False)]
        #self.get_logger().info(f"[Before filter] candidates: {free0}")
        #self.get_logger().info(f"[Before filter] dynamic_ratio = {self.dynamic_ratio:.3f}")
       

        if len(self.failed_thresholds) < 2:
            free = free0
        else:
            free = []
            for rid in free0:
                soc = self.batteries[rid]
                for pick_x, pick_y, drop_x, drop_y in batch_tasks:
                    rpos = np.array(self.robots[rid])
                    pick = np.array([pick_x, pick_y])
                    drop = np.array([drop_x, drop_y])
                    origin = np.array([0.0,0.0])
                    route_len = (np.linalg.norm(pick-rpos)
                               + np.linalg.norm(drop-pick)
                               + np.linalg.norm(origin-drop))
                    needed = route_len * self.dynamic_ratio
                    #self.get_logger().info(
                    #    f"R{rid}: route={route_len:.1f}m, soc={soc:.1f}%, "
                    #    f"threshold={self.dynamic_ratio:.3f}, needed={needed:.2f}%"
                    #)
                    if soc >= needed:
                        free.append(rid)
                        break
        #self.get_logger().info(f"[After filter] candidates: {free}")
        if not free:
            self.get_logger().warn("No free robots meet dynamic_ratio — postponing wave")
            return
        
        D = min(len(free), len(batch_tasks))
        if D == 0:
            return
        batch_tasks = batch_tasks[:D]
        self.current_batch = batch_tasks
        #self.get_logger().info(f"---Launching APSO (P={self.pso_P}, G={self.pso_G}, D={D}) "
        #                       f"with dynamic_ratio={self.dynamic_ratio:.3f} ---")
        
        P, G = self.pso_P, self.pso_G
        N = len(free)      
        lb = np.zeros((P, D))
        ub = np.ones((P, D)) * (N - 1)
        self.ids_in_batch = list(free)

        if self.apso is None or self.apso.D != D:
            self.get_logger().info(f"Initializing APSO (P={P}, G={G}, D={D})")
            self.apso = APSO(
                fitness=self._fitness,
                P=P, D=D, G=G,
                lb=lb, ub=ub
            )

        self._optimizing = True
        t0 = self.get_clock().now()
        future = self.opt_executor.submit(self.apso.opt)
        try:
            future.result(timeout=self.optimization_timeout)
        except self.TimeoutError:
            self.get_logger().warn(
                f"PSO optimization exceeded {self.optimization_timeout}s, "
                "using current best"
            )
        finally:
            self._optimizing = False
        t1 = self.get_clock().now()
        
        chosen_idxs = [int(round(x)) for x in self.apso.gbest_X[:D]]
        chosen_robots = [free[i] for i in chosen_idxs]
        chosen_socs = [self.batteries[r] for r in chosen_robots]
        #self.get_logger().info(f"APSO gbest_X = {self.apso.gbest_X[:D]}")
        #self.get_logger().info(f"Chosen robots = {chosen_robots} with SOCs = {chosen_socs}")
        self._publish_assignment(free, self.current_batch, t0, t1)  
        
    def _static_assign(self):
        if not self.tasks:
            return

        free = [r for r in self.robots
                if r not in self.busy and not self.robot_charging.get(r, False)]

        D = min(self.batch_size, len(self.tasks), len(free))
        if D == 0:
            return

        msg = TaskAssignment()
        batch_coords = []
        next_tid = self.next_task_id

        self.get_logger().info(
            f"[STATIC] assigning {D} task(s), free robots = {len(free)+D}"
        )
        
        for _ in range(D):
            px, py, dx, dy = self.tasks.popleft()

            rid = min(
                free,
                key=lambda r: (self.robots[r][0] - px) ** 2 +
                              (self.robots[r][1] - py) ** 2
            )
            free.remove(rid)
            self.busy.add(rid)

            msg.robot_id.append(rid)
            msg.task_id.append(next_tid)
            msg.target_x.extend([px, dx, 0.0])
            msg.target_y.extend([py, dy, 0.0])
            
            self.get_logger().info(
                f"[STATIC] Robot {rid} ← ({px:.2f},{py:.2f})→({dx:.2f},{dy:.2f})"
            )
            
            batch_coords.append((px, py, dx, dy))
            next_tid += 1

        self.assign_pub.publish(msg)
        start_ros = self.get_clock().now()
        for tid in msg.task_id:
            #start_ros = time.monotonic()
            self.task_start_times[tid] = start_ros
        if batch_coords:
            cost = sum(
                math.hypot(px, py) +
                math.hypot(dx - px, dy - py) +
                math.hypot(dx, dy)
                for px, py, dx, dy in batch_coords
            )
        else:
            cost = None

        
        self._write_stats(cost=cost)

    def _fitness(self, X: np.ndarray) -> np.ndarray:
        X = np.atleast_2d(X)
        P = X.shape[0]
        D = X.shape[1]

        ids    = self.ids_in_batch
        coords = np.array([self.robots[i] for i in ids])
        tasks  = list(itertools.islice(self.tasks, 0, D)) 
        origin = np.array([0.0, 0.0])

        fit = np.zeros(P)
        for p in range(P):
            used = set()
            cost = 0.0
            for d in range(D):
                idx = int(np.clip(round(X[p, d]), 0, len(ids) - 1))
                rid = ids[idx]

                if rid in used:
                    cost += self.collision_penalty
                used.add(rid)

                rpos  = coords[idx]
                pick  = np.array(tasks[d][:2])
                drop  = np.array(tasks[d][2:])
                tt = 0.0
                for p0,p1 in [(rpos, pick), (pick, drop), (drop, origin)]:
                    dist = np.linalg.norm(p1-p0)
                    v   = max(self.robots_speeds.get(rid,self.default_speed),0.1)
                    tt += dist/v
                cost += self.alpha * tt
                    
                route_len = np.linalg.norm(pick - rpos) \
                          + np.linalg.norm(drop - pick) \
                          + np.linalg.norm(origin - drop)
                energy_needed = route_len * self.dynamic_ratio
                soc = self.batteries[rid]
                if soc < energy_needed:
                    cost += self.risk_penalty
                 
            fit[p] = cost
        return fit

    def _publish_assignment(self, free, batch_tasks, start_time, end_time):
        elapsed = (end_time - start_time).nanoseconds / 1e6  # ms
        best_cost = float(self.apso.gbest_F)
        self.get_logger().info(
            f"APSO finished in {elapsed:.1f} ms, best cost = {best_cost:.2f}"
        )
        msg  = TaskAssignment()
        assigned = set()
        base_id = self.next_task_id
        
        batch_tasks = self.current_batch
        best = self.apso.gbest_X[:len(batch_tasks)]

        for d, x in enumerate(best):
            idx = int(np.clip(round(x), 0, len(free)-1))
            rid = free[idx]
            
            if rid in assigned:
                self.get_logger().debug(f"Duplicate {rid} → searching next free")
                rid = next((r for r in free if r not in assigned), None)
                if rid is None:
                    self.get_logger().warn("No free robots left, task postponed")
                    continue
            assigned.add(rid)
            
            pick_x, pick_y, drop_x, drop_y = batch_tasks[d]
            rpos = np.array(self.robots[rid])
            pick = np.array([pick_x, pick_y])
            drop = np.array([drop_x, drop_y])
            origin = np.array([0.0, 0.0])
            
            route_len = (
                np.linalg.norm(pick - rpos)
                + np.linalg.norm(drop - pick)
                + np.linalg.norm(origin - drop)
            )
            soc_start = self.batteries[rid]

            tid = base_id + d
            self.get_logger().info(f"TID {tid}")
            self.assignment_info[(rid, tid)] = (route_len, soc_start)
            
            msg.robot_id.append(rid)
            msg.task_id.append(tid)
            ddd = msg.task_id
            self.get_logger().info(f"DDD {ddd}")
            msg.target_x.extend([pick_x, drop_x, 0.0])
            msg.target_y.extend([pick_y, drop_y, 0.0])

            self.get_logger().info(
                f"Robot {rid} -> Task {d}: ({pick_x:.2f},{pick_y:.2f})->({drop_x:.2f},{drop_y:.2f})"
            )
        for t in batch_tasks:
            try:
                self.tasks.remove(t)
            except ValueError:
                pass

        self.assign_pub.publish(msg)
        start_ros = self.get_clock().now()
        for tid in msg.task_id:
            #start_ros = time.monotonic()
            self.task_start_times[tid] = start_ros
            self.get_logger().info(f"ADAPTIVE")
            self.get_logger().info(f"Task {tid} started at sim-time {start_ros.nanoseconds * 1e-9:.3f}s")
        self.busy.update(msg.robot_id)
        self.next_task_id = base_id + len(batch_tasks)
        self._write_stats(cost=best_cost)

    def finished_cb(self, msg):
        rid = msg.robot_id
        if rid in self.busy:
            self.busy.remove(rid)
            self.tasks_assigned += 1
            tid = msg.task_id
            #now_ros = time.monotonic()
            #now_ros = self.get_clock().now()
            #all_ros = self.task_start_times.pop(tid, None)
            #if all_ros is not None:
            #    dt = (now_ros - all_ros).nanoseconds * 1e-9
            #    self.task_durations.append(dt)
            #else:
            #    self.get_logger().warn(
            #        f"No start time recorded for tid={tid}; "
            #        f"existing keys: {list(self.task_start_times.keys())}"
            #    )
            
            if self.tasks_assigned % self.summary_block_size == 0:
                self.get_logger().info(
                    f">>> Wave {self.wave_id} done, flushing summary"
                )
                self._flush_wave_summary()
                self.wave_id += 1
                
            self._write_stats(cost=None)
            self.get_logger().info(f"Robot {rid} finished task, total done = {self.tasks_assigned}")
            
            if self.tasks_assigned >= self.max_tasks:
                self.get_logger().info(f"Reached {self.max_tasks} tasks — shutting down.")               
                self.pool.shutdown(wait=False)        
                raise KeyboardInterrupt
        else:
            self.get_logger().warn(f"Unexpected TaskFinished from R{rid}")
    
    def _flush_wave_summary(self):
        self.number += 5
        #if not self.task_durations:
        #     self.get_logger().warn("abort summary: no task durations to summarize")
        #     return
                    
        #total_time = sum(self.task_durations)
        tasks_in_block = self.summary_block_size
        #avg_time = total_time / tasks_in_block
        generated_total  = self.tasks_received
        planned_batch    = tasks_in_block + self.penalties
        sim_time         = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        avg_time = sim_time / self.number
        
        self.get_logger().info(
            f"Summary wave {self.wave_id}: generated={generated_total}, "
            f"completed_total={self.tasks_assigned}, in_block={planned_batch}/{tasks_in_block}, "
            f"penalties={self.penalties}, avg={avg_time:.2f}s"
        )
        
        write_header = not os.path.exists(self.summary_file) or os.path.getsize(self.summary_file) == 0
        header = "mode,number,generated,all_completed,planned_in_block,penalties,avg_time,sim_time,tasks_in_block\n"
        line = (
            f"{self.mode},{self.wave_id},{generated_total},{self.tasks_assigned},"
            f"{planned_batch},{self.penalties},"
            f"{avg_time:.2f},{sim_time:.2f},{tasks_in_block}\n"
        )
        if write_header:
            self.get_logger().info(f"Writing summary header to {self.summary_file}")
            self.get_logger().debug(header.strip())
        self.get_logger().info(f"Writing summary line for wave {self.wave_id}")
        self.get_logger().debug(line.strip())
        with open(self.summary_file, 'a', encoding='utf-8') as f:
            if write_header:
                f.write(header)
            f.write(line)
            
        self.task_durations.clear()
        self.penalties = 0

def main():
    rclpy.init()
    node = APSOAllocator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
