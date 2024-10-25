#!/usr/bin/bash

xhost +

if [ -z "$1" ]; then
  echo "No argument about the number of maps"
  exit 1
fi

max_value=$1

cd ../

home_path=$(pwd) 

source myenv/bin/activate

cd ros-gazebo-simulation/src/coverage_controller/src/

echo "Creating path..."
#waypoints file
for (( j=0; j<=max_value; j++ )); do
  las_path="$home_path/ros-gazebo-simulation/src/maps_file/las_files/map_$j.las"
  output_dir="$home_path/ros-gazebo-simulation/src/maps_file/mesh_files/map_$j"

  
  if [ -f "$output_dir/zigzag_path.txt" ]; then
    echo "zigzag_path.txt file already exists."
    continue
  fi

  python3 generate_waypoints.py "$las_path" --desired_height 60 --output_dir "$output_dir"
done
echo "Path Created!"

deactivate

cd ../../..

#launch simulation
for (( k=0; k<=max_value; k++ )); do
    world_path="/catkin_ws/src/maps_file/mesh_files/map_$k/map_$k.world"
    path_txt_path="/catkin_ws/src/maps_file/mesh_files/map_$k/zigzag_path.txt"
    caminho_output="/catkin_ws/src/maps_file/mesh_files/map_$k"
    model_file_path="/catkin_ws/src/maps_file/mesh_files/map_$k/mesh.dae"

    docker stop gazebo_uav_sim
    docker rm gazebo_uav_sim

    comando="source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch coverage_controller quadrotor_world.launch my_world_name:=$world_path my_waypoints:=$path_txt_path caminho_output_default:=$caminho_output model_file:=$model_file_path"
    echo "Launching simulation!"

    docker run --rm --privileged \
    --net=host \
    --ipc=host \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=0 \
    --env=DISPLAY \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=$HOME/.Xauthority:/root/.Xauthority:rw \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$PWD/src:/catkin_ws/src/:rw \
    --volume=$PWD/data:/data/:rw \
    --name gazebo_uav_sim \
    mavilab/gazebo-uav-sim:v1.0 \
    bash -c "$comando"
done

cd ../

source myenv/bin/activate

cd semantic-segmentation/

#semantic and depth images
for (( i=0; i<=max_value; i++ )); do
    echo "Generating semantic segmentation!"
    mesh_path="$home_path/ros-gazebo-simulation/src/maps_file/mesh_files/map_$i/mesh.dae"
    texture_path="$home_path/ros-gazebo-simulation/src/maps_file/mesh_files/map_$i/textura_semantic.jpeg"
    config_robot_path="$home_path/semantic-segmentation/robo_info.txt"
    camera_info_path="$home_path/semantic-segmentation/camera_info.txt"
    translation_file_path="$home_path/semantic-segmentation/world_translation.txt"
    input_file_path="$home_path/ros-gazebo-simulation/src/maps_file/mesh_files/map_$i/info-file/"
    output_file_path="$home_path/ros-gazebo-simulation/src/maps_file/mesh_files/map_$i/semantic-depth-segmentation/"

    python3 semantic-render.py "$input_file_path" "$camera_info_path" "$mesh_path" "$texture_path" "$config_robot_path" "$translation_file_path" "$output_file_path"
done

echo "Simulation Finished!"
deactivate
