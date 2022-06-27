cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(pcd_merge)

#new
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#was here
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

#new
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

#was here
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_merge pcd_merge.cpp)


#new
#ament_target_dependencies(pcd_merge rclcpp sensor_msgs message_filters Eigen3)

#was here
target_link_libraries(pcd_merge ${PCL_LIBRARIES})

#new
#ament_package()