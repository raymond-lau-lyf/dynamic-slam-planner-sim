cmake_minimum_required(VERSION 2.8)
project(odom_to_tf_and_path)

find_package(catkin REQUIRED nav_msgs roscpp tf std_srvs)

find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${catkin_INCLUDE_DIRS}

     /usr/include/eigen3/)

catkin_package()

add_executable(odom2tfAndPath src/odom2tfAndPath.cpp)
target_link_libraries(odom2tfAndPath ${catkin_LIBRARIES} ${PCL_LIBRARIES})
