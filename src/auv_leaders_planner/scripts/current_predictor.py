#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import netCDF4
import numpy as np
import torch
import torch.nn as nn
from sklearn.preprocessing import StandardScaler
from collections import deque
import sys
import os


class StudentNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.lstm = nn.LSTM(2, 64, num_layers=2, batch_first=True)
        self.norm = nn.LayerNorm(64)
        self.head = nn.Sequential(
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 3)
        )

    def forward(self, x):
        _, (h, _) = self.lstm(x)
        feat = self.norm(h[-1])
        return self.head(feat)


def generate_physics_data_for_scaler(u_series, v_series, total_steps=50000):
    data = []
    u_mean, u_std = np.mean(u_series), np.std(u_series)
    v_mean, v_std = np.mean(v_series), np.std(v_series)

    for _ in range(total_steps):
        u = np.random.normal(u_mean, u_std if u_std > 0 else 1.0)
        v = np.random.normal(v_mean, v_std if v_std > 0 else 1.0)
        w = np.random.normal(0, 0.1)
        q = np.random.normal(0, 0.1)
        r = np.random.normal(0, 0.1)

        uc = u
        vc = v
        wc = w

        data.append([u, v, w, q, r, uc, vc, wc])

    return np.array(data)


class CurrentPredictorNode(Node):
    def __init__(self):
        super().__init__('current_predictor')

        self.declare_parameter('nc_file', '/home/lu/paper1/src/network/expt_93_uv3z.nc4')
        self.declare_parameter('model_path', '/home/lu/paper1/src/network/general/Student_seed1314.pth')
        self.declare_parameter('seq_len', 30)

        self.nc_file = self.get_parameter('nc_file').value
        self.model_path = self.get_parameter('model_path').value
        self.seq_len = self.get_parameter('seq_len').value

        self.true_current_pub = self.create_publisher(Vector3, '/ocean_current', 10)
        self.pred_current_pub = self.create_publisher(Twist, '/predicted_current', 10)

        self.odom_sub = self.create_subscription(Odometry, '/auv0/odom', self.odom_callback, 10)

        self.input_buffer = deque(maxlen=self.seq_len)

        self.init_data_and_scalers()
        self.init_model()

        self.create_timer(1.0, self.playback_callback)
        self.current_step = 0

        self.get_logger().info("Current Predictor Node Initialized!")

    def init_data_and_scalers(self):
        try:
            ds = netCDF4.Dataset(self.nc_file, 'r')
            u_all = ds.variables['water_u'][:]
            v_all = ds.variables['water_v'][:]

            if u_all.ndim == 4:
                self.u_series = u_all[:, 0, 31, 31]
                self.v_series = v_all[:, 0, 31, 31]
            else:
                self.u_series = u_all[:, 31, 31]
                self.v_series = v_all[:, 31, 31]

            self.u_series = np.ma.filled(self.u_series, 0.0)
            self.v_series = np.ma.filled(self.v_series, 0.0)

            dummy_data = generate_physics_data_for_scaler(self.u_series, self.v_series)

            self.scaler_X = StandardScaler().fit(dummy_data[:, :5])
            self.scaler_Y = StandardScaler().fit(dummy_data[:, 5:])

            self.get_logger().info("Scalers fitted.")

        except Exception as e:
            self.get_logger().error(f"Data Init Error: {e}")
            sys.exit(1)

    def init_model(self):
        try:
            self.model = StudentNet()
            state_dict = torch.load(self.model_path, map_location='cpu')
            self.model.load_state_dict(state_dict)
            self.model.eval()
        except Exception as e:
            self.get_logger().error(f"Model Load Error: {e}")
            sys.exit(1)

    def playback_callback(self):
        idx = self.current_step % len(self.u_series)
        msg = Vector3()
        msg.x = float(self.u_series[idx])
        msg.y = float(self.v_series[idx])
        msg.z = 0.0
        self.true_current_pub.publish(msg)
        self.current_step += 1

    def odom_callback(self, msg):
        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        w = msg.twist.twist.linear.z
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        self.input_buffer.append([u, v, w, q, r])

        if len(self.input_buffer) == self.seq_len:
            self.predict()

    def predict(self):
        raw_input = np.array(self.input_buffer)
        norm_input = self.scaler_X.transform(raw_input)
        input_tensor = torch.tensor(norm_input, dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            output_tensor = self.model(input_tensor)

        pred_norm = output_tensor.numpy()
        pred_raw = self.scaler_Y.inverse_transform(pred_norm)[0]

        msg = Twist()
        msg.linear.x = float(pred_raw[0])
        msg.linear.y = float(pred_raw[1])
        msg.linear.z = float(pred_raw[2])

        self.pred_current_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
