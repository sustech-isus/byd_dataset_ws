#!/bin/bash
roscore &
sleep 5
echo "launch camera..."
roslaunch spinnaker_camera_driver 6cam_sync.launch  &
sleep 5
echo "launch hesai lidar..."
#roslaunch hesai_lidar hesai_lidar.launch  &
roslaunch pandar_pointcloud  Pandar128_points.launch &
#sleep 5
#roslaunch novatel_gps_driver gps.launch &
sleep 5
roslaunch huaxun_radar single.launch &
sleep 5
roslaunch infrared_camera_driver infrared_cam_driver.launch &
sleep 5
roslaunch rslidar_sdk start.launch &
sleep 5
#echo "start recording..."
#rosbag record -b 4096 --chunksize=1024 /fix /pandar_points /cameras/front/image_color /cameras/left/image_color /cameras/right/image_color /radar_pointcloud /radar_pointcloud_raw /radar_targets
#rosbag record -b 4096 --chunksize=1024 /pandar_points  /fix 
sleep 5
roslaunch byd_sdk run.launch &
sleep 5
roslaunch gps_driver run.launch

