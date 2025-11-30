# Autonomous Docking Using ArUco Markers in ROS2 Jazzy

## Overview
This project implements an autonomous docking system for a differential drive robot using ArUco markers in ROS2 Jazzy and Gazebo Sim (gz-sim).

## Packages
- `my_bot`: Contains the robot description (URDF), Gazebo simulation launch files, ArUco marker model, and the nodes for detection and docking.

## Prerequisites
- ROS2 Jazzy
- Gazebo Sim (gz-sim)
- `ros_gz` packages
- `opencv`

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
1. **Robot Simulation**: A differential drive robot with a camera is simulated in Gazebo. The URDF uses `gz-sim` plugins for differential drive and sensors.
2. **ArUco Detection**: The `aruco_detector` node subscribes to `/camera/image_raw`, detects the ArUco marker using OpenCV, estimates its pose relative to the camera, and publishes it to `/aruco_pose`.
3. **Docking Logic**: The `docking_node` node subscribes to `/aruco_pose`. It uses a proportional controller to align the robot with the marker and move towards it. It stops when the distance is approximately 0.25m, allowing for some small error.


## Potential Improvements

### 1. State Estimation & Sensor Fusion
- **Extended Kalman Filter (EKF)**: Integrate the `robot_localization` package to fuse wheel odometry with visual pose estimates from the ArUco detector. This would provide a continuous and robust state estimate (`odom` -> `base_link` tf) even when the marker is temporarily occluded or detection is noisy. It is also possible to implement better ArUco detection logics which would be robust to different conditions.

### 2. Motion planning, Finite State Machine
- The `docking_node` logic can be upgraded into a full state machine, which could follow some steps such as:
    - Search Strategy: A pre-defined search pattern when the marker is not visible.
    - Normal Approach: Planning in such a way that the robot docks exactly perpendicular to the AruCo marker.
    - Collision avoidance strategies in the planning algorithm

### 3. **Multi-Marker SLAM**: 
- I would have to implement handling the detection of multiple ArUco markers. With this, one can perform SLAM (Simultaneous Localization and Mapping) as well, allowing the robot to localize itself globally in the map frame rather than just relative to a single dock.


## Author
Pranav Debbad <br>
ED21B046
