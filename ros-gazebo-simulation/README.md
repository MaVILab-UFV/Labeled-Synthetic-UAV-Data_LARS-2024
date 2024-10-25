![ROS Package Index](https://img.shields.io/ros/v/noetic/ros)
# Gazebo UAV Simulation

This repository contains the necessary files to simulate a Cessna aircraft model in Gazebo with ROS integration. The simulation includes various sensors such as IMU, GPS, and a camera.

## Prerequisites

- Docker installed on your system
- Docker Compose installed on your system

## Getting Started

### 1. Clone the Repository

First, clone this repository to your local machine:

```bash
git clone git@github.com:MaVILab-UFV/2024-gazebo-uav.git
cd 2024-gazebo-uav
```

### 2. Build and Run the Docker Container
Ensure that you have Docker and Docker Compose installed. Then, use the provided docker-compose.yml to build and run the Docker container.

```bash
docker compose build
```

Start the simulation by running the following command:

```bash
./launch_sim.sh
```
This script will build and start the Docker container with the necessary configurations.

### 3. Generate path waypoint list from pointcloud
- This script takes the pointcloud and camera fov parameters and return a list of waypoints for the uav.
```sh
python3 generate_waypoints.py --help
```
```
usage: generate_waypoints.py [-h] [--scale SCALE] [--hfov HFOV] [--vfov VFOV]
                             [--desired_height DESIRED_HEIGHT] [--overlap OVERLAP]
                             file_path

Generate an optimized coverage path for a drone from a .las point cloud

positional arguments:
  file_path             Path to the .las file

optional arguments:
  -h, --help            show this help message and exit
  --scale SCALE         Scale of the points in .las file
  --hfov HFOV           Horizontal field of view of the camera in degrees
  --vfov VFOV           Vertical field of view of the camera in degrees
  --desired_height DESIRED_HEIGHT
                        Desired height of the camera
  --overlap OVERLAP     Overlap percentage for the grid
```
##### Example:
```sh
python3 generate_waypoints.py nuvem_filtrada.las --desired_height 50
```
![image](https://github.com/MaVILab-UFV/2024-gazebo-uav/assets/14208261/2baf9edd-3c13-4860-8b00-8ec67ab31402)

### 4. Launch the Simulation and the RViz
The launch_sim.sh script will automatically launch the simulation in Gazebo and RViz. If you need to manually start the simulation in RViz, open a new terminal and execute the following command:

#### OPTION 1: Hector Quad Simulation with coverage path planning

```sh
roslaunch coverage_controller quadrotor_world.launch
```

![image](https://github.com/MaVILab-UFV/2024-gazebo-uav/assets/14208261/e0dccd7a-3b7c-4d22-99c5-036ac023878e)

#### OPTION 2: Launch Cessna
```bash
roslaunch cessna_simulation cessna_world_rviz.launch
```
#### How to control the flight?
- ```w``` Increase thrust (+10 %)
- ```s``` Decrease thrust (-10 %)
- ```d``` Increase rudder angle (+1 degree)
- ```a``` Decrease rudder angle (-1 degree)
- ```Left-Key``` Left roll (+1 degree)
- ```Right-Key``` Right roll (+1 degree)
- ```Up-Key``` Pitch down (+1 degree)
- ```Down-Key``` Pitch up (+1 degree)
- ```1``` Preset for take-off
- ```2``` Preset for cruise
- ```3``` Preset for landing

![image](https://github.com/MaVILab-UFV/2024-gazebo-uav/assets/14208261/dc06d41f-cb5a-4d70-9317-0bfcd597acbb)

#### Topics
```bash
/altimeter
/altimeter/parameter_descriptions
/altimeter/parameter_updates
/cessna_c172/camera/camera_info
/cessna_c172/camera/image_raw
/cessna_c172/camera/image_raw/compressed
/cessna_c172/camera/image_raw/compressed/parameter_descriptions
/cessna_c172/camera/image_raw/compressed/parameter_updates
/cessna_c172/camera/image_raw/compressedDepth
/cessna_c172/camera/image_raw/compressedDepth/parameter_descriptions
/cessna_c172/camera/image_raw/compressedDepth/parameter_updates
/cessna_c172/camera/image_raw/theora
/cessna_c172/camera/image_raw/theora/parameter_descriptions
/cessna_c172/camera/image_raw/theora/parameter_updates
/cessna_c172/camera/parameter_descriptions
/cessna_c172/camera/parameter_updates
/clicked_point
/clock
/downward_cam/camera/camera_info
/downward_cam/camera/image
/downward_cam/camera/image/compressed
/downward_cam/camera/image/compressed/parameter_descriptions
/downward_cam/camera/image/compressed/parameter_updates
/downward_cam/camera/image/compressedDepth
/downward_cam/camera/image/compressedDepth/parameter_descriptions
/downward_cam/camera/image/compressedDepth/parameter_updates
/downward_cam/camera/image/theora
/downward_cam/camera/image/theora/parameter_descriptions
/downward_cam/camera/image/theora/parameter_updates
/downward_cam/parameter_descriptions
/downward_cam/parameter_updates
/fix
/fix/position/parameter_descriptions
/fix/position/parameter_updates
/fix/status/parameter_descriptions
/fix/status/parameter_updates
/fix/velocity/parameter_descriptions
/fix/velocity/parameter_updates
/fix_velocity
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/ground_truth/state
/initialpose
/joint_states
/magnetic
/magnetic/parameter_descriptions
/magnetic/parameter_updates
/move_base_simple/goal
/pressure_height
/raw_imu
/raw_imu/accel/parameter_descriptions
/raw_imu/accel/parameter_updates
/raw_imu/bias
/raw_imu/rate/parameter_descriptions
/raw_imu/rate/parameter_updates
/raw_imu/yaw/parameter_descriptions
/raw_imu/yaw/parameter_updates
/rosout
/rosout_agg
/tf
/tf_static
```


