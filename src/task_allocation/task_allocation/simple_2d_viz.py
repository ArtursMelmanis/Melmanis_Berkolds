#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import rclpy
from rclpy.node import Node
from typing import Sequence  
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.lines import Line2D
from robot_msgs.msg import RobotPos, TaskAssignment, TaskFinished
from std_msgs.msg import Int32

class Simple2DVisualizer(Node):
    def __init__(self):
        super().__init__('simple_2d_viz')
        self.start_time = self.get_clock().now()
        
        self.declare_parameter('collision_radius', 0.2)
        self.CR = self.get_parameter('collision_radius').value
        self.declare_parameter('blocked_center', [5.0, 5.0])
        self.declare_parameter('blocked_radius', 1.0)
        self.declare_parameter('block_period', 15.0)
        self.declare_parameter('block_duration', 8.0)
        self.blocked_center = tuple(self.get_parameter('blocked_center').value)
        self.blocked_radius = self.get_parameter('blocked_radius').value
        self.block_period   = self.get_parameter('block_period').value
        self.block_duration = self.get_parameter('block_duration').value

        self.robot_patches: dict[int, plt.Circle] = {}
        self.battery_text: dict[int, matplotlib.text.Text] = {}

        # подписки
        self.create_subscription(RobotPos,
                                 '/sim_positions',
                                 self.pos_cb, 10)
        self.create_subscription(TaskAssignment,
                                 '/task_assignments',
                                 self.task_cb, 10)
        self.create_subscription(TaskFinished,
                                 '/task_finished',
                                 self.finished_cb, 10)
        self.create_subscription(Int32,
                                 '/path_clear',
                                 self.clear_cb, 10)

        # интерактивный режим Matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        plt.show(block=False)

        self.ax.set_xlim(-10, 25)
        self.ax.set_ylim(-15, 15)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_title('HeRoSwarm 2D Simulacija')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')

        # зоны
        self.declare_parameter('zone_names',   ['A', 'B', 'C'])
        self.declare_parameter('zone_centers', [0.0, 0.0, 5.0, 0.0, -5.0, 0.0])
        self.declare_parameter('zone_radius',  1.0)

        names = self.get_parameter('zone_names').value
        flat  = self.get_parameter('zone_centers').value
        r     = self.get_parameter('zone_radius').value
    
        centers = [
            (flat[i], flat[i+1])
            for i in range(0, len(flat), 2)
        ]
        
        for name, (cx, cy) in zip(names, centers):
            circle = plt.Circle((cx, cy), r, color='gray', alpha=0.3)
            self.ax.add_patch(circle)
            self.ax.text(cx, cy, name,
                         ha='center', va='bottom',
                         fontsize=12, weight='bold')

        # список динамических путей
        self.robots: dict[int, tuple[float, float]] = {}   # rid → (x,y)
        self.markers: dict[int, Line2D] = {}               # rid → кружок
        self.cmap = cm.get_cmap('tab20', 20)               # 20 уник. цветов
        self.legend_lines: list[Line2D] = []               # для легенды
        self.pick_proxy = Line2D([0], [0], marker='s', color='grey',
                                 linestyle='None', label='pick')
        self.drop_proxy = Line2D([0], [0], marker='^', color='grey',
                                 linestyle='None', label='drop')
        self.legend_added = False

        # rid → анимация
        self.active: dict[int, dict] = {}
        # rid → данные будущего пути
        self.pending: dict[int, tuple[list[float], list[float]]] = {}
        # линии маршрутов
        self.paths: list[Line2D] = []
        
        self.barrier_patch = plt.Circle(
            self.blocked_center, 
            radius=0.0,
            fill=True,
            facecolor='red',
            edgecolor='darkred',
            linewidth=1, 
            alpha=0.2,
            zorder=10
        )
        self.ax.add_patch(self.barrier_patch)
        self.ax.legend([], [], loc='upper right')

    def pos_cb(self, msg: RobotPos) -> None:
        now = self.get_clock().now()
        elapsed = now - self.start_time
        sim_time = elapsed.nanoseconds * 1e-9
        
        t = sim_time % self.block_period
        if t < self.block_duration:
            self.barrier_patch.set_radius(self.blocked_radius)
        else:
            self.barrier_patch.set_radius(0.0)
            
        for odom in msg.robot_pos:
            rid = int(odom.child_frame_id)
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            battery = odom.twist.twist.linear.z

            # запоминаем позицию
            self.robots[rid] = (x, y)

            # Если появился новый робот — создаём кружок и добавляем в легенду
            color = self.cmap(rid % 20)
            if rid not in self.robot_patches:
                #color = self.cmap(rid % 20)
                #(ln,) = self.ax.plot(x, y, 'o', markersize=6, color=color)
                #self.markers[rid] = ln
                circ = plt.Circle(
                    (x, y), 
                    radius=self.CR,
                    edgecolor=color,
                    facecolor=color,
                    alpha=0.5,
                    zorder=1
                )
                self.robot_patches[rid] = circ
                self.ax.add_patch(circ)
                # подпись в легенду
                #lg = Line2D([], [], color=color, label=f'R{rid}')
                #self.legend_lines.append(lg)
                #self.ax.legend(handles=self.legend_lines, loc='upper right')
                lg = Line2D([], [], marker='o', color=color, linestyle='',
                            markersize=6, label=f'R{rid}')
                self.legend_lines.append(lg)
                self.ax.legend(handles=self.legend_lines, loc='upper right')
            else:
                self.robot_patches[rid].center = (x, y)
            
            txt_str = f"{battery:.0f}"
            if rid not in self.battery_text:
                txt = self.ax.text(x+0.2, y+0.2, txt_str,
                                   fontsize=8, color='black')
                self.battery_text[rid] = txt
            else:
                txt = self.battery_text[rid]
                txt.set_position((x+0.2, y+0.2))
                txt.set_text(txt_str)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _draw_path(self, rid: int, x0: float, y0: float,
               txs: Sequence[float], tys: Sequence[float]) -> dict:
        # стартовая точка = актуальная позиция робота
        x1, x2, x3 = txs
        y1, y2, y3 = tys
        color = self.cmap(rid % 20)
        
        # рисуем новый маршрут и маркеры
        ln, = self.ax.plot([x0, x1, x2, x3],
                           [y0, y1, y2, y3],
                           '-', lw=2, color=color, label='_nolegend_')
        sq, = self.ax.plot(x1, y1, 's', ms=8, color=color,
                           linestyle='None', mec='black', mew='0.5', label='_nolegend_')     # pick
        tr, = self.ax.plot(x2, y2, '^', ms=8, color=color,
                           linestyle='None', mec='black', mew='0.5', label='_nolegend_')     # drop
        ar  = self.ax.annotate('', xy=(x3, y3), xytext=(x2, y2),
                         arrowprops=dict(arrowstyle='->', color=color, lw=1.5))

        seg_len = (
            np.hypot(x1 - x0, y1 - y0) +
            np.hypot(x2 - x1, y2 - y1) +
            np.hypot(x3 - x2, y3 - y2)
        ) or 1.0  # защита от деления на 0

        return {
            'points': [(x0, y0), (x1, y1), (x2, y2), (x3, y3)],
            'progress': 0.0,
            'total_len': seg_len,
            'artists': [ln, sq, tr, ar],
        }
        

    def task_cb(self, msg: TaskAssignment) -> None:
        """Рисуем маршрут: от тек. позиции робота → pick → drop → origin."""
        xs, ys = msg.target_x, msg.target_y
        if len(xs) % 3 != 0 or len(xs) == 0:
            self.get_logger().warn("TaskAssignment target arrays malformed")
            return

        for i in range(0, len(xs), 3):
            rid = msg.robot_id[i // 3]
            txs: List[float] = xs[i:i+3]
            tys: List[float] = ys[i:i+3]

            # ── если робот исполнял задачу → убрать старые художники ──────────
            if rid in self.active:
                self.pending[rid] = (xs[i:i+3], ys[i:i+3])
                return

            x0, y0 = self.robots.get(rid, (0.0, 0.0))
            self.active[rid] = self._draw_path(rid, x0, y0, txs, tys)

        # ── обновляем легенду без дубликатов ──────────────────────────────────
        handles, labels = self.ax.get_legend_handles_labels()
        uniq: dict[str, Any] = {}
        for h, lab in zip(handles, labels):
            if lab == '_nolegend_':      # пропускаем служебные
                continue
            uniq[lab] = h

        # гарантированно добавляем proxy‑pick / proxy‑drop
        uniq['pick'] = self.pick_proxy
        uniq['drop'] = self.drop_proxy
        self.ax.legend(uniq.values(), uniq.keys(), loc='upper right')

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def finished_cb(self, msg) -> None:
        rid: int = msg.robot_id
        changed: bool = False
        
        if rid in self.active:
            for art in self.active[rid]['artists']:
                art.remove()
            del self.active[rid]
            changed = True
        
        if rid in self.pending:           # нарисовать сохранённый путь
            txs, tys = self.pending.pop(rid)
            x0, y0 = self.robots.get(rid, (0.0, 0.0))
            self.active[rid] = self._draw_path(rid, x0, y0, txs, tys)
            changed = True

        if changed:
            self.fig.canvas.draw_idle()
        

    #  5.  helper – линейная интерполяция по полилинии
    @staticmethod
    def _interp_along(points, t):
        """t ∈ [0,1] → точка вдоль ломаной pick-drop-return."""
        segs = [np.hypot(x2 - x1, y2 - y1)
                for (x1, y1), (x2, y2) in zip(points[:-1], points[1:])]
        total = sum(segs)

        dist = t * total
        for (x1, y1), (x2, y2), L in zip(points[:-1], points[1:], segs):
            if dist > L:
                dist -= L
            else:
                ratio = dist / L if L else 0
                return x1 + ratio * (x2 - x1), y1 + ratio * (y2 - y1)
        return points[-1]  # fallback

    def clear_cb(self, msg):
        rid = int(msg.data)
        if rid in self.active:
            for art in self.active[rid]['artists']:
                art.remove()
            del self.active[rid]
            self.fig.canvas.draw_idle()

def main():
    rclpy.init()
    node = Simple2DVisualizer()
    try:
        rclpy.spin(node)
    finally:
        plt.ioff()
        plt.close(node.fig)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
