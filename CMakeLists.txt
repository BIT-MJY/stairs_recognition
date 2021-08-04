cmake_minimum_required(VERSION 2.8.3)
project(stairs_recognition)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  pcl_conversions
)

set(PCL_DIR "/usr/share/pcl-1.8")
set(OpenCV_DIR "/usr/share/OpenCV") 
# find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
	# ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs pcl_conversions
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)




add_executable(stairs_recognition src/stairs_recognition.cpp)
target_link_libraries(stairs_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})







