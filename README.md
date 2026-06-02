# A pysical simulator for ECA A9 
## Paper Name: A Hierarchical Game-Theoretic Framework with Spatio-Temporal Flow Perception for Underactuated AUV Cooperative Operations

A multi-agent formation control demo for ECA A9 AUVs, integrating planning, tracking control, and learned ocean current prediction in Gazebo Harmonic with Project DAVE.

## Quick Start

```bash
ros2 launch auv_leaders_planner leaders_planner.launch.py gui:=true rviz:=true
```

Spawning three ECA A9 AUVs and drives them to a global goal in formation.

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic
- [Project DAVE](https://github.com/Field-Robotics-Lab/dave) (hydrodynamics / buoyancy plugins)
- PyTorch, netCDF4, scikit-learn (for current prediction)

Set `dave_prefix` in `leaders_planner.launch.py` to your DAVE install path, or export `DAVE_WS_PATH`.

## Packages

| Package | Role |
|---------|------|
| `auv_leaders_planner` | Multi-agent formation planner + ocean current predictor |
| `tracking_control` | SF local tracking controller + thruster allocator |
| `model` | ECA A9 URDF, Gazebo world, sensor macros |

## Features

- **Formation Planner** — Leader-follower formation using consensus-based auxiliary variables, with kinematic constraints and deadzone braking.
- **Tracking Controller** — Sliding-mode-based surge / pitch / yaw controller with LOS guidance, ocean-current feedforward compensation, and buoyancy restoration.
- **Thruster Allocator** — Force-to-actuator mapping for single propeller + X-rudder fins.
- **Current Predictor** — Lightweight LSTM network predicting 3-DOF ocean current from AUV motion, trained on NetCDF current data.

## Disclaimer

Built for a personal academic paper. Based on Project DAVE and UUV Simulator. Cite both the original sources and this repository if used.
