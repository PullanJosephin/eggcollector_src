#!/bin/bash
map_dir="${HOME}/catkin_ws/src/turn_on_wheeltec_robot/map"
map_name="mymap"
 
#finish trajectory
#rosservice call /finish_trajectory 0
 
#save pbsream
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"
 
#pbstream to map
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name \
-resolution=0.05