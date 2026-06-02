# ECA A9 Multi-AUV Formation Demo

A Gazebo Harmonic multi-AUV simulation demo for ECA A9, including formation planning, tracking control, and ocean-current prediction.

---

## Tech Stack

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-8A2BE2)
![Project DAVE](https://img.shields.io/badge/Project-DAVE-0A7E8C)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)
![PyTorch](https://img.shields.io/badge/PyTorch-Model-EE4C2C?logo=pytorch&logoColor=white)
![NetCDF4](https://img.shields.io/badge/NetCDF4-Data-4C9A2A)

---

## Quick Start

```bash
cd ~/your_ws
colcon build --packages-select auv_leaders_planner tracking_control model
source install/setup.bash
ros2 launch auv_leaders_planner leaders_planner.launch.py gui:=true rviz:=true
```

---

## Project Structure

```text
.
├── src/
│   ├── auv_leaders_planner/          # formation planner + current predictor
│   │   ├── launch/
│   │   │   └── leaders_planner.launch.py
│   │   ├── src/
│   │   │   └── leaders_planner.cpp
│   │   ├── scripts/
│   │   │   ├── current_predictor.py
│   │   │   └── publish_n4c_current.py
│   │   ├── config/
│   │   │   └── leaders_params.yaml
│   │   └── msg/
│   │       └── AuxVar.msg
│   │
│   ├── tracking_control/             # local tracking controller + allocator
│   │   ├── src/
│   │   │   ├── sf_local_tracking.cpp
│   │   │   └── thruster_allocator.cpp
│   │   ├── config/
│   │   │   └── vehicle_params.yaml
│   │   └── launch/
│   │       └── start_control.launch.py
│   │
│   ├── model/                        # ECA A9 model + Gazebo world
│   │   ├── urdf/
│   │   │   ├── eca_a9_dave.urdf.xacro
│   │   │   └── sensor.xacro
│   │   ├── worlds/
│   │   │   └── dave_ocean_waves_fixed.world
│   │   ├── launch/
│   │   │   └── start_dave.launch.py
│   │   ├── mesh/
│   │   └── sensor/
│   │
│   └── network/                      # current field + trained model
│       ├── general/
│       │   └── Student_seed1314.pth
│       └── expt_93_uv3z.nc4
│
├── README.md
└── .gitignore
```

---

## Model Sources

- **ECA A9 URDF / mesh**: adapted and cleaned from the ECA A9 description in [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator)
- **Hydrodynamics / buoyancy / thruster / sensor plugins**: provided by [Project DAVE](https://github.com/Field-Robotics-Lab/dave)
- **Ocean current data**: loaded from NetCDF current-field files

> The current launch file expects a local DAVE installation. Before running, check the `dave_prefix` in `leaders_planner.launch.py`, or set the `DAVE_WS_PATH` environment variable.

---

## Key Parameters

| Parameter | Value | Note |
|-----------|-------|------|
| AUV mass (URDF) | 100.8 kg | collision geometry ≈ cylinder 2.35m × ⌀0.23m |
| AUV mass (controller) | 100.0 kg | tuned vehicle-model parameter |
| Displacement volume | 0.098 m³ | near-neutral buoyancy |
| COB–COM distance (BG) | 0.08 m | COB at +0.06 m, COM at −0.02 m |
| Surge damping | −8.0 | controller-side linear damping |
| Pitch damping | −20.0 | controller-side linear damping |
| Yaw damping | −32.0 | controller-side linear damping |

---

## FAQ

### 1. Why does Gazebo or the world fail to launch?
Make sure that:
- ROS 2 Jazzy and Gazebo Harmonic are installed correctly
- the DAVE workspace is built
- `dave_prefix` or `DAVE_WS_PATH` points to the correct DAVE installation path

### 2. Why does the current predictor report a missing file?
Make sure the following files exist:
- `src/network/expt_93_uv3z.nc4`
- `src/network/general/Student_seed1314.pth`

### 3. Why does the AUV still show slight vertical oscillation in Gazebo?
The model combines the buoyancy plugin, the hydrodynamics plugin, and controller-side restoring-moment compensation. Small oscillations are usually related to controller gains, damping values, COB/COM relative position, and ocean-current disturbance.

---

## Notice

This repository is provided for research and educational use. Please check the original licenses of Project DAVE, UUV Simulator, and any third-party assets before reuse.
