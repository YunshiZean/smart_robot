#!/usr/bin/env bash

current_dir=$(pwd)
current_datetime=$(date "+%Y%m%d%H%M%S")
map_file_name=${current_dir}"/map_"${current_datetime}".pbstream"
echo ${map_file_name}

rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${map_file_name}'}"

# map_file_name=${current_dir}"/map_20250210102954.pbstream"
# save pbstream to ros map (pgm and yaml)
resolution=0.05
sleep 1
if [ -f "${map_file_name}" ];then
    pgm_file_name=${current_dir}"/map_"${current_datetime}
    rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${pgm_file_name} -pbstream_filename=${map_file_name} -resolution=${resolution}
    echo "Map was saved successfully."    
else
    echo "Failed! ${map_file_name} does not exist."
fi

