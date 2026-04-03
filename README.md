# ECA A9 AUV Simulation Model

This repository contains the simulation model of the ECA A9 Autonomous Underwater Vehicle (AUV) and the corresponding underwater environment (world) for Gazebo. It is designed to work seamlessly with the Project DAVE (Aquatic Virtual Environment) framework.

## Dependencies
This package relies on the **Project DAVE Framework** for underwater hydrodynamics and sensor plugins. 
Before using this model, ensure that the DAVE workspace (e.g., `dave_ws`) is built and sourced correctly. The main launch files will automatically attempt to source the DAVE environment path if it is located at `/home/lu/dave_ws/install`.

## Features

### AUV Model (`eca_a9_dave.urdf.xacro`)
- **Hydrodynamics**: Integrated with Fossen's 6-DOF hydrodynamic model via `gz-sim-hydrodynamics-system`.
- **Actuators**: 
  - 1x Main propeller (thruster) for surge speed.
  - 4x Controllable fins for pitch and yaw steering (with Lift-Drag physics).
- **Odometry**: Publishes ground-truth state via `gz-sim-odometry-publisher-system`.
- **Sensors Payload (Optional / Reserved)**:
  - The model includes predefined Xacro macros for various sensors such as a 3D Pose Sensor, DVL, Side Scan Sonar, Pressure Sensor, IMU, Forward-looking Camera, GPS, and Chemical Concentration Sensor. These can be enabled by uncommenting the `sensor_snippets.xacro` include in the main URDF.

### Simulation World (`dave_ocean_waves_fixed.world`)
- Implements realistic underwater physics including buoyancy and gravity.
- Includes a detailed sand heightmap and coastal water environment.
- Configured lighting and shadow rendering for underwater visual fidelity.

## Package Structure
- `config/`: Parameter files (e.g., `ros_gz_bridge.yaml`) for ROS-Gazebo message bridging.
- `launch/`: Launch files to spawn the environment (`start_dave.launch.py`).
- `mesh/`: Visual and collision meshes (DAE/STL) for the ECA A9 body, propeller, and fins.
- `sensor/`: Modular Xacro macros for various underwater sensors.
- `urdf/`: Core Xacro files defining the AUV's kinematics, dynamics, and Gazebo plugins.
- `worlds/`: Gazebo world files defining the underwater simulation scene.

## Usage
This package is primarily launched via the main planning/control launch files in the workspace (e.g., `leaders_planner.launch.py`). It handles spawning the world and the AUV instances with their respective namespaces, enabling multi-agent (swarm) simulations.

## Disclaimer & License Information 
The basic code associated with the physical world and AUV model under Gazebo Harmonic. This repository is constructed **exclusively for a personal academic paper**. It represents an optimization and refactoring of the ECA A9 model based on existing open-source frameworks, primarily the **Project DAVE** and the **UUV Simulator**. 

If you require to download or use this code for academic research or commercial purposes, **you must properly cite the original source and this repository**. The author assumes no liability or responsibility for any disputes, damages, or consequences arising from the unauthorized or improper use of this code.

This repository contains the simulation model of the ECA A9 Autonomous Underwater Vehicle (AUV) and the corresponding underwater environment (world) for Gazebo. It is designed to work seamlessly with the Project DAVE (Aquatic Virtual Environment) framework.
