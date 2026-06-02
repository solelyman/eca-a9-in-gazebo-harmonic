#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import netCDF4
import numpy as np
import time
import os

class NetCDFPublisher(Node):
    def __init__(self):
        super().__init__('netcdf_current_publisher')
        
        # Declare parameters
        self.declare_parameter('file_path', '/home/lu/paper1/src/network/expt_93_uv3z.nc4')
        self.declare_parameter('u_var', 'water_u')  # Eastward velocity variable name
        self.declare_parameter('v_var', 'water_v')  # Northward velocity variable name
        self.declare_parameter('time_step', 1.0) # Time step in seconds between data points
        self.declare_parameter('scale_factor', 1.0)
        self.declare_parameter('lat_idx', 31) # Latitude index (center of 63)
        self.declare_parameter('lon_idx', 31) # Longitude index (center of 63)
        
        self.publisher_ = self.create_publisher(Vector3, '/ocean_current', 10)
        
        file_path = self.get_parameter('file_path').get_parameter_value().string_value
        if not file_path:
            self.get_logger().error('No file_path provided!')
            return

        if not os.path.exists(file_path):
             self.get_logger().error(f'File not found: {file_path}')
             return

        try:
            self.nc = netCDF4.Dataset(file_path, 'r')
            self.get_logger().info(f'Opened NetCDF file: {file_path}')
            
            u_var = self.get_parameter('u_var').get_parameter_value().string_value
            v_var = self.get_parameter('v_var').get_parameter_value().string_value
            lat_idx = self.get_parameter('lat_idx').get_parameter_value().integer_value
            lon_idx = self.get_parameter('lon_idx').get_parameter_value().integer_value
            
            # Data shape is (time, depth, lat, lon) -> (148, 1, 63, 63)
            # We take the first depth layer (0) and the specified lat/lon indices
            
            self.u_data = self.nc.variables[u_var][:]
            self.v_data = self.nc.variables[v_var][:]
            
            self.get_logger().info(f"Variable '{u_var}' shape: {self.u_data.shape}")
            
            # Check dimensions and slice
            if self.u_data.ndim == 4:
                # (time, depth, lat, lon)
                self.u_series = self.u_data[:, 0, lat_idx, lon_idx]
                self.v_series = self.v_data[:, 0, lat_idx, lon_idx]
            elif self.u_data.ndim == 3:
                # (time, lat, lon)
                self.u_series = self.u_data[:, lat_idx, lon_idx]
                self.v_series = self.v_data[:, lat_idx, lon_idx]
            else:
                # Assume 1D time series or handle other cases
                self.u_series = self.u_data.flatten()
                self.v_series = self.v_data.flatten()
                
            self.data_len = len(self.u_series)
            self.current_idx = 0
            
            timer_period = self.get_parameter('time_step').get_parameter_value().double_value
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(f'Started publishing with {self.data_len} data points, interval: {timer_period}s')
            
        except Exception as e:
            self.get_logger().error(f'Failed to read NetCDF file: {e}')

    def timer_callback(self):
        if self.current_idx >= self.data_len:
            self.current_idx = 0 # Loop or stop? Let's loop.
            self.get_logger().info('Restarting data playback')
            
        # Convert masked array to float if necessary
        u_val = float(self.u_series[self.current_idx])
        v_val = float(self.v_series[self.current_idx])
        
        # Check for masked values (fill value)
        if np.ma.is_masked(u_val):
            u_val = 0.0
        if np.ma.is_masked(v_val):
            v_val = 0.0

        scale = self.get_parameter('scale_factor').get_parameter_value().double_value
        
        msg = Vector3()
        msg.x = u_val * scale
        msg.y = v_val * scale
        msg.z = 0.0
        
        self.publisher_.publish(msg)
        # Optional: Log every 10 steps to reduce noise
        if self.current_idx % 10 == 0:
            self.get_logger().info(f'Published Current: u={msg.x:.3f}, v={msg.y:.3f} (Idx: {self.current_idx})')
        
        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = NetCDFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
