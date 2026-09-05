#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

bags=(
  /home/dev/Dataset/vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle.bag
  /home/dev/Dataset/vicon/2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle.bag
  /home/dev/Dataset/calib/2025-10-31-16-05-17.bag
  /home/dev/ws_uwb/src/SFUISE/dataset/ISAS-Walk1.bag
  /home/dev/ws_uwb/src/SFUISE/dataset/ISAS-Walk2.bag
  /home/dev/ws_uwb/src/SFUISE/dataset/ISAS-Walk3.bag
  /home/dev/ws_uwb/src/awesome-uwb-localization/bag/data_example.bag
  /home/dev/Dataset/ntu_day01/ntu_day_01_mid70.bag
  /home/dev/Dataset/ntu_day01/ntu_day_01_ltpb.bag
)

for bag_path in "${bags[@]}"; do
  printf 'BAG=%s\n' "$bag_path"
  sha256sum "$bag_path"
  rosbag info --yaml "$bag_path"
  printf 'ROSBAG_INFO_EXIT_CODE=%d\n' "$?"
done
