#!/usr/bin/env python3
import os
import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from auv_leaders_planner.msg import AuxVar


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        self.declare_parameter('duration', 120.0)
        self.declare_parameter('output_dir', '/home/lu/paper1/outputs/metrics')
        self.declare_parameter('model_name', 'default')
        self.declare_parameter('agent_ids', [0, 1, 2])
        self.declare_parameter('rate_hz', 10.0)

        self.duration = float(self.get_parameter('duration').value)
        self.output_dir = self.get_parameter('output_dir').value
        self.model_name = self.get_parameter('model_name').value
        self.agent_ids = list(self.get_parameter('agent_ids').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.start_time = None
        self.last_positions = {}
        self.last_goals = {}
        self.last_aux = {}

        for agent_id in self.agent_ids:
            odom_topic = f'/auv{agent_id}/odom'
            goal_topic = f'/auv{agent_id}/final_goal'
            aux_topic = f'/swarm_comms/auv{agent_id}'

            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, aid=agent_id: self.odom_callback(msg, aid),
                10
            )
            self.create_subscription(
                Point,
                goal_topic,
                lambda msg, aid=agent_id: self.goal_callback(msg, aid),
                10
            )
            self.create_subscription(
                AuxVar,
                aux_topic,
                lambda msg, aid=agent_id: self.aux_callback(msg, aid),
                10
            )

        self.rows = []
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def odom_callback(self, msg, agent_id):
        p = msg.pose.pose.position
        self.last_positions[agent_id] = (float(p.x), float(p.y), float(p.z))

    def goal_callback(self, msg, agent_id):
        self.last_goals[agent_id] = (float(msg.x), float(msg.y), float(msg.z))

    def aux_callback(self, msg, agent_id):
        z = msg.z
        s = msg.s
        rho = msg.rho
        self.last_aux[agent_id] = (
            (float(z.x), float(z.y), float(z.z)),
            (float(s.x), float(s.y), float(s.z)),
            (float(rho.x), float(rho.y), float(rho.z)),
        )

    def tick(self):
        now = self.get_clock().now()
        if self.start_time is None:
            if self.last_positions:
                self.start_time = now
            else:
                return
        t = (now - self.start_time).nanoseconds / 1e9
        if t >= self.duration:
            self.write_csv()
            rclpy.shutdown()
            return

        if not self.last_positions:
            return

        formation_errors = []
        z_norms = []
        s_norms = []
        rho_norms = []

        for agent_id in self.agent_ids:
            if agent_id in self.last_positions and agent_id in self.last_goals:
                px, py, pz = self.last_positions[agent_id]
                gx, gy, gz = self.last_goals[agent_id]
                formation_errors.append(math.sqrt((px - gx) ** 2 + (py - gy) ** 2 + (pz - gz) ** 2))
            if agent_id in self.last_aux:
                z, s, rho = self.last_aux[agent_id]
                z_norms.append(math.sqrt(z[0] ** 2 + z[1] ** 2 + z[2] ** 2))
                s_norms.append(math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2))
                rho_norms.append(math.sqrt(rho[0] ** 2 + rho[1] ** 2 + rho[2] ** 2))

        formation_error_avg = sum(formation_errors) / len(formation_errors) if formation_errors else float('nan')
        z_norm_avg = sum(z_norms) / len(z_norms) if z_norms else float('nan')
        s_norm_avg = sum(s_norms) / len(s_norms) if s_norms else float('nan')
        rho_norm_avg = sum(rho_norms) / len(rho_norms) if rho_norms else float('nan')

        row = [t, formation_error_avg, z_norm_avg, s_norm_avg, rho_norm_avg]
        for agent_id in self.agent_ids:
            px = py = pz = float('nan')
            if agent_id in self.last_positions:
                px, py, pz = self.last_positions[agent_id]
            z = s = rho = (float('nan'), float('nan'), float('nan'))
            z_val = s_val = rho_val = float('nan')
            if agent_id in self.last_aux:
                z, s, rho = self.last_aux[agent_id]
                z_val = math.sqrt(z[0] ** 2 + z[1] ** 2 + z[2] ** 2)
                s_val = math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2)
                rho_val = math.sqrt(rho[0] ** 2 + rho[1] ** 2 + rho[2] ** 2)
            row.extend([
                px, py, pz,
                z[0], z[1], z[2], z_val,
                s[0], s[1], s[2], s_val,
                rho[0], rho[1], rho[2], rho_val
            ])

        self.rows.append(row)

    def write_csv(self):
        model_dir = os.path.join(self.output_dir, self.model_name)
        os.makedirs(model_dir, exist_ok=True)
        out_path = os.path.join(model_dir, 'convergence_metrics.csv')

        header = ['time', 'formation_error_avg', 'z_norm_avg', 's_norm_avg', 'rho_norm_avg']
        for agent_id in self.agent_ids:
            header.extend([
                f'auv{agent_id}_x', f'auv{agent_id}_y', f'auv{agent_id}_z',
                f'auv{agent_id}_z_x', f'auv{agent_id}_z_y', f'auv{agent_id}_z_z', f'auv{agent_id}_z_norm',
                f'auv{agent_id}_s_x', f'auv{agent_id}_s_y', f'auv{agent_id}_s_z', f'auv{agent_id}_s_norm',
                f'auv{agent_id}_rho_x', f'auv{agent_id}_rho_y', f'auv{agent_id}_rho_z', f'auv{agent_id}_rho_norm'
            ])

        with open(out_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.rows)


def main():
    rclpy.init()
    node = MetricsRecorder()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
