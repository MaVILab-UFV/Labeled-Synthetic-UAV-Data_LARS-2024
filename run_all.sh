#!/bin/bash

# URLs tif_files
tif_images=(
  'https://data.geo.admin.ch/ch.swisstopo.swissimage-dop10/swissimage-dop10_2021_2722-1124/swissimage-dop10_2021_2722-1124_0.1_2056.tif'
  #'https://data.geo.admin.ch/ch.swisstopo.swissimage-dop10/swissimage-dop10_2021_2636-1185/swissimage-dop10_2021_2636-1185_0.1_2056.tif'
  #'https://data.geo.admin.ch/ch.swisstopo.swissimage-dop10/swissimage-dop10_2021_2637-1185/swissimage-dop10_2021_2637-1185_0.1_2056.tif'
  #'https://data.geo.admin.ch/ch.swisstopo.swissimage-dop10/swissimage-dop10_2021_2638-1185/swissimage-dop10_2021_2638-1185_0.1_2056.tif'
  #'https://data.geo.admin.ch/ch.swisstopo.swissimage-dop10/swissimage-dop10_2021_2636-1184/swissimage-dop10_2021_2636-1184_0.1_2056.tif'
)

#URLs las_files
las_files=(
  'https://data.geo.admin.ch/ch.swisstopo.swisssurface3d/swisssurface3d_2020_2722-1124/swisssurface3d_2020_2722-1124_2056_5728.las.zip'
  #'https://data.geo.admin.ch/ch.swisstopo.swisssurface3d/swisssurface3d_2020_2637-1193/swisssurface3d_2020_2637-1193_2056_5728.las.zip'
  #'https://data.geo.admin.ch/ch.swisstopo.swisssurface3d/swisssurface3d_2019_2638-1193/swisssurface3d_2019_2638-1193_2056_5728.las.zip'
  #'https://data.geo.admin.ch/ch.swisstopo.swisssurface3d/swisssurface3d_2019_2639-1193/swisssurface3d_2019_2639-1193_2056_5728.las.zip'
  #'https://data.geo.admin.ch/ch.swisstopo.swisssurface3d/swisssurface3d_2019_2637-1192/swisssurface3d_2019_2637-1192_2056_5728.las.zip'
)

#set the size of the world 0-10000 and the min coordinates
x_min=0
y_min=0
world_size=10000

home_path=$(pwd)

mkdir -p ros-gazebo-simulation/src/maps_file
mkdir -p ros-gazebo-simulation/src/maps_file/las_files
mkdir -p ros-gazebo-simulation/src/maps_file/images_tif_files
mkdir -p ros-gazebo-simulation/src/maps_file/mesh_files
for map in "${!las_files[@]}"; do
  mkdir -p ros-gazebo-simulation/src/maps_file/mesh_files/map_$map
  mkdir -p ros-gazebo-simulation/src/maps_file/mesh_files/map_$map/images-file
  mkdir -p ros-gazebo-simulation/src/maps_file/mesh_files/map_$map/info-file
  mkdir -p ros-gazebo-simulation/src/maps_file/mesh_files/map_$map/semantic-depth-segmentation
  mkdir -p ros-gazebo-simulation/src/maps_file/mesh_files/map_$map/camera_info_file
done

#dowload and rename tif
download_and_rename() {
  url=$1
  dest_folder=$2
  new_name=$3
  dest_file="$dest_folder/$new_name"

  if [ -f "$dest_file" ]; then
    echo "File $dest_file already exists. Skipping download."
  else
    wget -O "$dest_file" "$url"
    if [ $? -eq 0 ]; then
      echo "Downloaded and renamed $url to $dest_file"
    else
      echo "Failed to download $url"
    fi
  fi
}

#extract and rename zip
extract_and_rename_zip() {
  file_path=$1
  extract_to=$2
  if unzip -o "$file_path" -d "$extract_to"; then
    echo "Extracted $file_path to $extract_to"
    extracted_file=$(unzip -Z1 "$file_path")
    new_name="${file_path%.zip}"
    mv "$extract_to/$extracted_file" "$new_name"
    echo "Renamed $extract_to/$extracted_file to $new_name"
  else
    echo "File $file_path is not a valid zip file"
  fi
}

#dowload and rename tif
for i in "${!tif_images[@]}"; do
  new_name="map_$i.tif"
  download_and_rename "${tif_images[$i]}" "ros-gazebo-simulation/src/maps_file/images_tif_files" "$new_name"
done

extract and rename las_files
for j in "${!las_files[@]}"; do
  url="${las_files[$j]}"
  zip_name="map_$j.las.zip"
  zip_path="$home_path/ros-gazebo-simulation/src/maps_file/las_files/$zip_name"
  
  if [ -f "$zip_path" ]; then
    echo "File $zip_path already exists. Skipping download."
  else
    wget -O "$zip_path" "$url"

    if [ $? -eq 0 ]; then
      extract_and_rename_zip "$zip_path" "ros-gazebo-simulation/src/maps_file/las_files"
    else
      echo "Failed to download $url"
    fi
  fi
done

#Virtualenv
if [ ! -d "myenv" ]; then
  virtualenv myenv
  echo "Virtualenv created."
  source myenv/bin/activate
  echo "Virtualenv activated."
  pip install -r requirements.txt
else
  echo "Virtualenv already exists. Skipping creation."
  source myenv/bin/activate
  echo "Virtualenv activated."
fi

cd map-generation/

# Create map
for k in "${!las_files[@]}"; do
  las_path="$home_path/ros-gazebo-simulation/src/maps_file/las_files/map_$k.las"
  tif_path="$home_path/ros-gazebo-simulation/src/maps_file/images_tif_files/map_$k.tif"
  output_path="$home_path/ros-gazebo-simulation/src/maps_file/mesh_files/map_$k"
  world_file="$output_path/map_$k.world"

  echo "Creating World..."

  if [ -f "$world_file" ]; then
    echo "File $world_file already exists. Skipping generation."
    continue
  fi

  python3 gera_mapa.py "$tif_path" "$las_path" "$output_path" "$x_min" "$y_min" "$world_size"
  output_path_world="/catkin_ws/src/maps_file/mesh_files/map_$k"

  #.world file
  world_content="<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<sdf version=\"1.4\">
  <world name=\"default\">
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.5 0.5 0.5 1</background>
      <shadows>false</shadows>
    </scene>
    <physics type=\"ode\">
      <real_time_update_rate>12000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>10</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
    </physics>
    <light type=\"directional\" name=\"directional_light_1\">
      <pose>0 20 20 0.1 0.1 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>1 1 1 1</specular>
      <attenuation>
        <range>300</range>
      </attenuation>
      <direction>0.1 0.1 -1</direction>
      <cast_shadows>false</cast_shadows>
    </light>
    <model name=\"landscape\">
      <link name=\"landscape_link\">
        <pose>0 0 0 0 0 0</pose>
        <collision name=\"landscape_collision\">
          <geometry>
            <mesh>
              <uri></uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name=\"landscape\">
          <geometry>
            <mesh>
              <uri>file://$output_path_world/mesh.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <cast_shadows>false</cast_shadows>
        </visual>
      </link>
      <static>true</static>
    </model>
  </world>
</sdf>"

  echo "$world_content" > "$world_file"

  echo "World Created!"
done

deactivate
echo "virtualenv closed" 
cd ..

cd ros-gazebo-simulation/
map_size=${#tif_images[@]}
./launch_sim.sh map_size
