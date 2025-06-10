# Cartesian Path Planning with the UR10e Robotic Arm 

This repository demonstrates Cartesian path planning using trapezoidal velocity profiles on a modified UR10e 6-DOF robotic arm, which includes an additional prismatic lift joint at the base. The system is visualized in Rviz and Gazebo using MoveIt, and it enforces orientation constraints to keep the end-effector perpendicular to the motion plane throughout its trajectory mimicking the action of spray painting a wall.

## Demo Videos

1. Rviz Visualization

<div align="center">
  <video src=https://private-user-images.githubusercontent.com/72541517/453325344-3647d4b9-4ab1-4772-b78f-2b29ff6c8d9a.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDk1MzkzNjMsIm5iZiI6MTc0OTUzOTA2MywicGF0aCI6Ii83MjU0MTUxNy80NTMzMjUzNDQtMzY0N2Q0YjktNGFiMS00NzcyLWI3OGYtMmIyOWZmNmM4ZDlhLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MTAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjEwVDA3MDQyM1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTYyMmY3YjZiNTE0ODBjZWE0Yjk4MTEwZDNkMWM1ZmQ5NmJiNmJmZTNiZDQ1YzhmZmUzYmFhZDk4MWM4MDIyYTYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.YvcRpUXpOQ4vGdrMQp-PnwmJnWnPT_k-Hgs-1kXEIpY />
</div>

2. Gazebo Simulation

<div align="center">
  <video src=https://private-user-images.githubusercontent.com/72541517/453326416-47b58714-21fc-4b9b-a72f-f5c9f9eb04b5.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDk1Mzk1NTgsIm5iZiI6MTc0OTUzOTI1OCwicGF0aCI6Ii83MjU0MTUxNy80NTMzMjY0MTYtNDdiNTg3MTQtMjFmYy00YjliLWE3MmYtZjVjOWY5ZWIwNGI1Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MTAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjEwVDA3MDczOFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWFmZmMzMWUxNDcyMzE0MTI1Y2UzMThlNTAwNDNmMzJiNjBhZmUzZWVjMzljYWVjMzdjZTU2MTA4YmQxY2Y4MmImWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.2c4nQdXrH2i0UI4guRNroSp0iKjmHwv6If_Y9WE0Rdk />
</div>

3. Velocity Plots

<div align="center">
  <video src=https://private-user-images.githubusercontent.com/72541517/453326762-db6f98d2-ba42-41f9-82c7-af98568faee6.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDk1Mzk2NDEsIm5iZiI6MTc0OTUzOTM0MSwicGF0aCI6Ii83MjU0MTUxNy80NTMzMjY3NjItZGI2Zjk4ZDItYmE0Mi00MWY5LTgyYzctYWY5ODU2OGZhZWU2Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MTAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjEwVDA3MDkwMVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWY0NGE3ZWE3OWU4Yzk5NjFmODFkMGEzMjVlMDEwY2E5NWFlOGI1ODg5ZWJmMTAzNzQ4ZGIyNmJjODU2NGIxODgmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.FOezw8yq8dS9PKzg3eunkOd6TH2FTaCQXXjLps1eEGE />
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

1. Launch the Full Simulation - Start all necessary nodes, robot state, visualization, and planning interfaces:

```
ros2 launch ur10e_mod ur10e_mod_move_it.launch.py
```

2. Load Waypoints from YAML File - Waypoints are defined in a waypoints.yaml file inside the path_planning_demo/config directory. Load them into the planner:

```
ros2 service call /plan_cartesian_path ur10e_mod_interfaces/srv/PlanCartesianPath "{from_yaml: true}"
```

3. Specify Waypoints Manually - You can also specify waypoints inline via the service call:

```
ros2 service call /plan_cartesian_path ur10e_mod_interfaces/srv/PlanCartesianPath "{waypoints: [{position: {x: 0.5, y: 0.7, z: 1.5}, orientation: {x: -0.707, y: 0.0, z: 0.0, w: 0.707}}]}"
```

4. Execute Planned Trajectory - Once the trajectory is visualized and confirmed in Rviz, execute it in the Gazebo environment:

```
ros2 service call /execute_planned_trajectory std_srvs/srv/Trigger
```

## Features

- Orientation constraint enforcement to maintain tool perpendicularity against a plane.
- Integrated planning support for both the arm and the base prismatic lift joint.
- Support for both Rviz-based visualization and Gazebo physics simulation.