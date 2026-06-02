# ECA A9 Multi-AUV Formation Demo

Multi-agent formation control simulation for ECA A9 AUVs in Gazebo Harmonic, integrating a consensus-based formation planner, sliding-mode tracking controller, and data-driven ocean current prediction.

## Quick Start

```bash
# Build
cd ~/your_ws
colcon build --packages-select auv_leaders_planner tracking_control model
source install/setup.bash

# Launch (3 AUVs, Gazebo + RViz)
ros2 launch auv_leaders_planner leaders_planner.launch.py gui:=true rviz:=true
```

## Dependencies

- **ROS 2 Jazzy** + Gazebo Harmonic (`ros_gz_sim`)
- **[Project DAVE](https://github.com/Field-Robotics-Lab/dave)** — underwater hydrodynamics, buoyancy, and thruster plugins (`gz-sim-hydrodynamics-system`, `gz-sim-buoyancy-system`, etc.)
- **Python packages**: `torch`, `netCDF4`, `scikit-learn`, `numpy` (current predictor)

> The launch file expects DAVE installed at `/home/lu/dave_ws/install`. Set `dave_prefix` in `leaders_planner.launch.py` to your path, or export `DAVE_WS_PATH`.

## Packages

| Package | Description |
|---------|-------------|
| `auv_leaders_planner` | Formation planner (`leader_planner_node`), current predictor node, NetCDF ocean-current playback |
| `tracking_control` | SF local tracking controller (`sf_local_tracking`), thruster-to-actuator allocator |
| `model` | ECA A9 URDF/Xacro model with Gazebo plugins, simulation world, mesh assets, optional sensor macros |

## Architecture

```
current_predictor ──→ /predicted_current (Twist)
       │
       └──→ /ocean_current (Vector3) ──→ Gazebo (flow velocity)

planner ──→ reference_state ──→ sf_controller ──→ cmd_vel ──→ thruster_allocator ──→ Gazebo (thrust + fins)
   ↑                                                                                        │
   └──── /swarm_comms/auv{id} (neighbor aux vars) ←─────────────────────────────────────────┘
                                                                                             │
                                                                                     ros_gz_bridge
                                                                                        │
                                                                                 Gazebo odometry
```

- **Planner** generates reference trajectories via consensus-based auxiliary variables with kinematic limits and deadzone logic
- **Controller** tracks the reference using surge/pitch/yaw sliding-mode control, LOS guidance, online ocean-current compensation, and buoyancy-restoring moment
- **Allocator** maps force/torque to single main propeller (angular velocity) and four X-rudder fins (position commands)
- **Current predictor** runs a lightweight LSTM (StudentNet) trained on NetCDF flow fields, producing 3-DOF current predictions from the leader's velocity history

## File Structure

```
auv_leaders_planner/
  config/leaders_params.yaml     — formation gains, deadzone, kinematic limits
  launch/leaders_planner.launch.py — main entry point
  msg/AuxVar.msg                  — inter-agent communication message
  scripts/current_predictor.py    — LSTM-based ocean current node
  scripts/publish_n4c_current.py  — raw NetCDF playback node
  src/leaders_planner.cpp          — formation planner

tracking_control/
  config/vehicle_params.yaml      — model parameters (mass, damping, COB, gains)
  src/sf_local_tracking.cpp       — LOS + sliding-mode tracking controller
  src/thruster_allocator.cpp      — force-to-angular-velocity & X-rudder mixing

model/
  urdf/eca_a9_dave.urdf.xacro     — ECA A9 kinematics, dynamics, Gazebo plugins
  launch/start_dave.launch.py     — Gazebo world spawner
  worlds/dave_ocean_waves_fixed.world — underwater simulation environment
  sensor/*.xacro                  — optional sensor payload macros (IMU, DVL, Camera, etc.)
```

## Key Parameters

| Parameter | Value | Note |
|-----------|-------|------|
| AUV mass (URDF) | 100.8 kg | Collision geometry ≈ cylinder 2.35m × ⌀0.23m |
| AUV mass (controller) | 100.0 kg | Tuned vehicle-model parameter |
| Displacement volume | 0.098 m³ | ~98.5 L, near-neutral buoyancy |
| COB—COM distance (BG) | 0.08 m | COB at +0.06m, COM at −0.02m in body frame |
| Surge linear damping (d_u) | −8.0 N/(m/s) | |
| Pitch damping (d_q) | −20.0 N·m/(rad/s) | |
| Yaw damping (d_r) | −32.0 N·m/(rad/s) | |

## Attribution

This repository is part of the paper *[Your Paper Title]*. The simulation model and underwater environment are adapted from:

- **ECA A9 URDF & mesh** — derived from the [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) ECA A9 description
- **Hydrodynamics, buoyancy, thruster, and sensor plugins** — from [Project DAVE](https://github.com/Field-Robotics-Lab/dave) (`gz-sim-hydrodynamics-system`, `gz-sim-buoyancy-system`, `gz-sim-lift-drag-system`, etc.)
- **Ocean current data** — NetCDF fields from operational ocean model outputs

If you use this code in academic work, please cite both the original sources above and this repository.

## License

This project is provided for academic and research purposes. The ECA A9 mesh assets, plugin binaries, and hydrodynamic parameter sets originate from their respective open-source projects and retain their original licenses. All original controller, planner, and prediction code is released for reproducibility of the associated paper. The author assumes no liability for any use of this software.
