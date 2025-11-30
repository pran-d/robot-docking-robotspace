# Autonomous Docking Using ArUco Markers in ROS2 Jazzy

## Overview
This project implements an autonomous docking system for a differential drive robot using ArUco markers in ROS2 Jazzy and Gazebo Sim (gz-sim).

## Packages
- `my_bot`: Contains the robot description (URDF), Gazebo simulation launch files, ArUco marker model, and the nodes for detection and docking.

## Prerequisites
- ROS2 Jazzy
- Gazebo Sim (gz-sim)
- `ros_gz` packages
- `opencv-python`
- `cv_bridge`

## Build Instructions
1. Navigate to the workspace root.
2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the workspace:
   ```bash
   colcon build
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Solution
To launch the simulation, ArUco detector, and docking node all at once:
```bash
ros2 launch my_bot launch_docking.launch.py
```

Alternatively, you can run them separately:
1. Launch the simulation:
   ```bash
   ros2 launch my_bot launch_sim.launch.py
   ```
2. Run the ArUco detector:
   ```bash
   ros2 run my_bot aruco_detector
   ```
3. Run the docking node:
   ```bash
   ros2 run my_bot docking_node
   ```

## Approach
1. **Robot Simulation**: A differential drive robot with a camera is simulated in Gazebo Sim. The URDF uses `gz-sim` plugins for differential drive and sensors.
2. **ArUco Detection**: The `aruco_detector` node subscribes to `/camera/image_raw`, detects the ArUco marker (ID 0, DICT_4X4_50) using OpenCV, estimates its pose relative to the camera, and publishes it to `/aruco_pose`.
3. **Docking Logic**: The `docking_node` node subscribes to `/aruco_pose`. It uses a proportional controller to align the robot with the marker and move towards it. It stops when the distance is approximately 0.2m.

## Challenges
- **Gazebo Sim Migration**: Migrating from Gazebo Classic plugins to Gazebo Sim systems required updating the URDF and Launch files.
- **Model Resources**: Creating a custom ArUco marker model and ensuring `GZ_SIM_RESOURCE_PATH` is set correctly so Gazebo Sim can find it.

## Improvements
- Implement a PID controller for smoother docking.
- Add state machine to handle "Search", "Approach", "Docked" states.
- Handle multiple markers.
