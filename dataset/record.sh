#!/bin/bash
roscore &
sleep 5
echo "launch camera..."
roslaunch spinnaker_camera_driver 6cam.launch  &
sleep 5
echo "launch hesai lidar..."
roslaunch hesai_lidar hesai_lidar.launch  &
#sleep 5
#roslaunch novatel_gps_driver gps.launch &
sleep 5
roslaunch huaxun_radar single.launch &
#echo "start recording..."
#rosbag record -b 4096 --chunksize=1024 /fix /pandar_points /cameras/front/image_color /cameras/left/image_color /cameras/right/image_color /radar_pointcloud /radar_pointcloud_raw /radar_targets
#rosbag record -b 4096 --chunksize=1024 /pandar_points  /fix /inspva

