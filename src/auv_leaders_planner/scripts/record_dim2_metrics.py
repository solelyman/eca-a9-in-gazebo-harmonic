#!/usr/bin/env python3
import os
import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Dim2Recorder(Node):
    def __init__(self):
        super().__init__('dim2_metrics_recorder')
        self.declare_parameter('duration', 120.0)
        self.declare_parameter('output_dir', '/home/lu/paper1/outputs/dim2')
        self.declare_parameter('model_name', 'default')
        self.declare_parameter('agent_ids', [0, 1, 2])
        self.declare_parameter('rate_hz', 10.0)

        self.duration = float(self.get_parameter('duration').value)
        self.output_dir = self.get_parameter('output_dir').value
        self.model_name = self.get_parameter('model_name').value
        self.agent_ids = list(self.get_parameter('agent_ids').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.start_time = self.get_clock().now()
        self.last_positions = {}
        self.last_refs = {}
        self.last_cmd = {}
        self.last_tick_time = None

        self.iae = {aid: 0.0 for aid in self.agent_ids}
        self.energy = {aid: 0.0 for aid in self.agent_ids}

        for aid in self.agent_ids:
            odom_topic = f'/auv{aid}/odom'
            ref_topic = f'/auv{aid}/reference_state'
            cmd_topic = f'/auv{aid}/cmd_vel'

            self.create_subscription(Odometry, odom_topic,
                                     lambda msg, a=aid: self.odom_cb(msg, a), 10)
            self.create_subscription(Odometry, ref_topic,
                                     lambda msg, a=aid: self.ref_cb(msg, a), 10)
            self.create_subscription(Twist, cmd_topic,
                                     lambda msg, a=aid: self.cmd_cb(msg, a), 10)

        self.rows = []
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def odom_cb(self, msg, aid):
        p = msg.pose.pose.position
        self.last_positions[aid] = (float(p.x), float(p.y), float(p.z))

    def ref_cb(self, msg, aid):
        p = msg.pose.pose.position
        self.last_refs[aid] = (float(p.x), float(p.y), float(p.z))

    def cmd_cb(self, msg, aid):
        self.last_cmd[aid] = (float(msg.linear.x), float(msg.angular.y), float(msg.angular.z))

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        if t >= self.duration:
            self.write_csv()
            rclpy.shutdown()
            return

        if self.last_tick_time is None:
            self.last_tick_time = now
            return

        dt = (now - self.last_tick_time).nanoseconds / 1e9
        self.last_tick_time = now

        err_norms = []
        iae_vals = []
        effort_vals = []

        for aid in self.agent_ids:
            if aid in self.last_positions and aid in self.last_refs:
                px, py, pz = self.last_positions[aid]
                rx, ry, rz = self.last_refs[aid]
                err = math.sqrt((px - rx) ** 2 + (py - ry) ** 2 + (pz - rz) ** 2)
                self.iae[aid] += err * dt
                err_norms.append(err)
                iae_vals.append(self.iae[aid])
            if aid in self.last_cmd:
                tu, tq, tr = self.last_cmd[aid]
                effort = tu * tu + tq * tq + tr * tr
                self.energy[aid] += effort * dt
                effort_vals.append(self.energy[aid])

        err_avg = sum(err_norms) / len(err_norms) if err_norms else float('nan')
        iae_avg = sum(iae_vals) / len(iae_vals) if iae_vals else float('nan')
        energy_avg = sum(effort_vals) / len(effort_vals) if effort_vals else float('nan')

        row = [t, err_avg, iae_avg, energy_avg]
        for aid in self.agent_ids:
            row.extend([
                self.iae.get(aid, float('nan')),
                self.energy.get(aid, float('nan'))
            ])
        self.rows.append(row)

    def write_csv(self):
        model_dir = os.path.join(self.output_dir, self.model_name)
        os.makedirs(model_dir, exist_ok=True)
        out_path = os.path.join(model_dir, 'tracking_effort.csv')

        header = ['time', 'err_norm_avg', 'iae_avg', 'energy_avg']
        for aid in self.agent_ids:
            header.extend([f'auv{aid}_iae', f'auv{aid}_energy'])

        with open(out_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.rows)


def main():
    rclpy.init()
    node = Dim2Recorder()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
