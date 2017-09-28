cmake_minimum_required(VERSION 2.8.3)
project(sicktoolbox_wrapper2)

find_package(
    catkin REQUIRED COMPONENTS 
    sensor_msgs roscpp sicktoolbox2
)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS sensor_msgs roscpp sicktoolbox2
  DEPENDS system_lib Threads
)

include_directories(${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

subdirs(standalone ros)
