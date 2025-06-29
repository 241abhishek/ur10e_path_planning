# Cartesian Path Planning with the UR10e Robotic Arm 

This repository demonstrates Cartesian path planning using trapezoidal velocity profiles on a modified UR10e 6-DOF robotic arm, which includes an additional prismatic lift joint at the base. The system is visualized in Rviz and Gazebo using MoveIt, and it enforces orientation constraints to keep the end-effector perpendicular to the motion plane throughout its trajectory mimicking the action of spray painting a wall.

## Demo Videos

1. Gazebo Simulation

<div align="center">
  <video src=https://private-user-images.githubusercontent.com/72541517/460334657-84af209e-4323-4c09-ae62-b99989d96e5b.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NTEyMjI0MTEsIm5iZiI6MTc1MTIyMjExMSwicGF0aCI6Ii83MjU0MTUxNy80NjAzMzQ2NTctODRhZjIwOWUtNDMyMy00YzA5LWFlNjItYjk5OTg5ZDk2ZTViLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MjklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjI5VDE4MzUxMVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTBmZDg5N2Q0ZWY0YmNkNTAyZDA4OWVmOWQwNWQyMWVhODZlZDM1Y2Y3NDU5Zjc3ZDNhYzY2YTk3YTFhYmJlOWYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.cgltp9raDL212dK_UjKHj0qnNG-g1pLQaSnPOxPCGGk />
</div>

2. Rviz Visualization

<div align="center">
  <video src=https://private-user-images.githubusercontent.com/72541517/460335052-07d26dc3-926a-4606-a1c2-e402df3db998.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NTEyMjI0ODIsIm5iZiI6MTc1MTIyMjE4MiwicGF0aCI6Ii83MjU0MTUxNy80NjAzMzUwNTItMDdkMjZkYzMtOTI2YS00NjA2LWExYzItZTQwMmRmM2RiOTk4Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MjklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjI5VDE4MzYyMlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWU0Y2YyZTI2M2Y3NTY5ZjAxY2Y1NDhmNzg1ZGEwN2QyZDUxMjQ5YTI4NWZkYWYxMzY4YmU3MWJmMDdjNTM4ZmImWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0._AH1UMX6u1-N3I_zwWofNXJcjmnj8lR9xiGqC0AnATc />
</div>

3. Velocity Plot (Constant Velocity Cartesian Motion for the End-Effector using Trapezoidal Velocity Profiles)

<div align="center">
  <video src=https://private-user-images.githubusercontent.com/72541517/460331169-406cc548-58f9-47d4-97dd-e7c230ebfdae.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NTEyMjA5NDksIm5iZiI6MTc1MTIyMDY0OSwicGF0aCI6Ii83MjU0MTUxNy80NjAzMzExNjktNDA2Y2M1NDgtNThmOS00N2Q0LTk3ZGQtZTdjMjMwZWJmZGFlLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MjklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjI5VDE4MTA0OVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTkwNTVlOWRmNWM3YjIzNzZkYmY0MWZmN2JkZTUwNWZiOWZmMTY3M2I3Y2U3MmRmNTFiNmMwYmJjZjRhZWMxYzEmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.pxUCuBLfHLJJPmDwqBMiZD0EqOwkgI4j9e24AwjgdcU />
</div>

## Packages Overview
[path_planning_demo](path_planning_demo):
Contains ROS 2 nodes responsible for planning Cartesian paths, loading waypoints, and executing trajectories using service interfaces.

[ur10e_mod](ur10e_mod):
Includes URDF, robot description, controller configuration, and launch files for spawning the modified UR10e with an additional prismatic joint.

[ur10e_mod_interfaces](ur10e_mod_interfaces):
Custom ROS 2 interface definitions, including the PlanCartesianPath service used for specifying and loading waypoints.

[ur10e_mod_moveit_config](ur10e_mod_moveit_config):
MoveIt configuration package tailored for the modified UR10e. Contains planning pipelines, kinematics settings, and controller integration.

## Quickstart

1. Install all dependencies using the following command (replace ros-distro if required):

```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-ur-controllers ros-jazzy-ur-description ros-jazzy-moveit
```

2. Build and source the workspace:

```
mkdir -p ros_ws/src
cd ros_ws/src
git clone git@github.com:241abhishek/ur10e_path_planning.git
cd ..
colcon build
source install/setup.bash
```

3. Launch the Full Simulation - Start all necessary nodes, robot state, visualization, and planning interfaces:

```
ros2 launch ur10e_mod ur10e_mod_move_it.launch.py
```

4. Load Waypoints from YAML File - Waypoints are defined in a waypoints.yaml file inside the path_planning_demo/config directory (by default). Set the "waypoints_file_path" param first and then load them into the planner. The snippet below assumes execution is carried out from the ROS workspace root:

```
ros2 param set /cartesian_planner waypoints_file_path src/path_planning_demo/config/waypoints.yaml
ros2 service call /plan_pilz_cartesian_path ur10e_mod_interfaces/srv/PlanCartesianPath "{from_yaml: true}"
```

5. Specify Waypoints Manually - You can also specify waypoints inline via the service call:

```
ros2 service call /plan_pilz_cartesian_path ur10e_mod_interfaces/srv/PlanCartesianPath "{waypoints: [{position: {x: 0.5, y: 0.75, z: 1.5}, orientation: {x: -0.707, y: 0.0, z: 0.0, w: 0.707}}]}"
```

6. Execute Planned Trajectory - Once the trajectory has been successfully planned, execute it in the Gazebo environment:

```
ros2 service call /execute_planned_trajectory std_srvs/srv/Trigger
```

## Features

- Orientation constraint enforcement to maintain tool perpendicularity against a plane.
- Constant velocity cartesian motion using trapezoidal velocity profiles.
- Integrated planning support for both the arm and the base prismatic lift joint.
- Support for both Rviz-based visualization and Gazebo physics simulation.