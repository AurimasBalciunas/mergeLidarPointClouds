# mergeLidarPointClouds
WIP: Script to merge left, front, and right LIDAR point clouds for MIT-PITT-RW AutoIndy

Currently only works on one set of point clouds till binning is figured out

source /opt/ros2/galactic/setup.bash

source ~/dev/joint-bvs-ws/joint-bvs-ws/install/setup.sh

Run python3 bag_to_pcd_lidar.py --source ./<ros bag file.db3> 

  (this will make one set of matching timestamps in split_results)
cd Merger

go into src/pcd_merge.cpp and rename the hardcoded set of three hardcoded PCDs to whatever yours were saved as by the bag_to_pcd_lidar script

cmake .

make

./pcd_merge 

 (this won't save anything, need to figure out how to save PointCloud2 as .bin in saveXYZI)
