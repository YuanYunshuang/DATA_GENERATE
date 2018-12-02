cmake_minimum_required(VERSION 2.8.3)
project(data_recorder)
add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(PCL  REQUIRED )
find_package(OpenCV  REQUIRED)

## Declare a catkin package
catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(data_recorder src/data_recorder.cpp)
target_link_libraries(data_recorder ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


