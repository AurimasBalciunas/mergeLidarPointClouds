# mergeLidarPointClouds

Script to asynchronously (without needing to run the rosbag) merge left, front, and right point clouds for MIT-PITT-AutoIndy. Major advantage is one does not need to run the rosbag for it to work. In combination with it being a completely command-line solution (unlike rviz2), should allow us to take advantage of our servers' computation power.

Takes in rosbags, outputs merged .bin and .pcd files.

### Setup instructions:

Clone repo

    source /opt/ros2/galactic/setup.bash

    source ~/dev/joint-bvs-ws/joint-bvs-ws/install/setup.sh

    cd path_to_this_Reop

Create five folders : split_bins, split_pcds, merged_bins, merged_pcds, text_results

    python3 bag_to_pcd_lidar.py --source ./path_to_ros_file.db3

*Note: if you would like to make more than 1000 merged pointclouds, increase the binLimit variable*
  
    cd Merger

    cmake .

    make

move the pcd_merge executable to the main repo folder

    ./pcd_merge

Your merged pcd files will be in merged_pcds, and the corresponding merged bin representations will be in merged_bins

