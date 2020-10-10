# RoboND-Where-am-I?

This project, provides implementation of adaptive monte carlo localizatin (AMCL) algorithm in ROS.

# Map Setup

Using the pgm_create_map package a pgm map file needs to be created from the my_world.world file.

Open a terminal, run gzerver with the map file:
```
gzserver src/pgm_map_creator/world/<YOUR GAZEBO WORLD FILE>
```
Open another terminal, launch the request_publisher node
```
roslaunch pgm_map_creator request_publisher.launch
```
The map will be located in the map folder of the pgm_map_creator

The parameters in the launch file of the node were set to
```
<?xml version="1.0" ?>
<launch>
  <arg name="map_name" default="map" />
  <arg name="save_folder" default="$(find pgm_map_creator)/maps" />
  <arg name="xmin" default="-20" />
  <arg name="xmax" default="12" />
  <arg name="ymin" default="-12" />
  <arg name="ymax" default="12" />
  <arg name="scan_height" default="5" />
  <arg name="resolution" default="0.01" />

  <node pkg="pgm_map_creator" type="request_publisher" name="request_publisher" output="screen" args="'($(arg xmin),$(arg ymax))($(arg xmax),$(arg ymax))($(arg xmax),$(arg ymin))($(arg xmin),$(arg ymin))' $(arg scan_height) $(arg resolution) $(arg save_folder)/$(arg map_name)">
  </node>
</launch>
```

- Running the Scripts
Run the following commands below in separate terminals:

```
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash 
```
# Project Launch
Launch the world in Gazebo and RViz:
```
cd /home/workspace/catkin_ws/
roslaunch my_robot world.launch
```

Launch the AMCL node for localization:
```
roslaunch my_robot amcl.launch
```

# Project Structure


```
 Project                          # Where Am I Project
    ├── my_robot                       # my_robot package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   │   ├── amcl.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── config                     # config folder for global and local planners
    │   │   ├── base_local_planner_params.yaml
    │   │   ├── costmap_common_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   ├── local_costmap_params.yaml
    │   ├── world                      # world folder for world files
    │   │   ├── myworld.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── pgm_map_creator                # pgm_map_creator package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── request_publisher.launch
    │   ├── world                      # world files that should be converted
    │   │   ├── my_world.world
    │   ├── maps                       # pgm image files
    │   │   ├── map.pgm
    ├── ball_chaser                    # ball_chaser package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    └──
```

# Results:

![Screenshot (55)](https://user-images.githubusercontent.com/49041896/95640153-635fd200-0a69-11eb-9a14-1bc93f975302.png)


The follwing results shows that the robot Pose Estimation. At first the particles shown in red are spread uniformly around the robot. Moving the robot further around, the particles get more centerd. 
![Screenshot (54)](https://user-images.githubusercontent.com/49041896/95640279-06185080-0a6a-11eb-90b1-b232da52ab32.png)

