cmake_minimum_required(VERSION 3.10)
project(radar_slam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    msgs_radar
    message_generation
    nuscenes2bag
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)


message(Eigen: ${EIGEN3_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_generation nuscenes2bag
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS
)

add_executable(radarOdometry 
    src/radarOdometry.cpp
    src/preprocess.cpp)
target_link_libraries(radarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(radarOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
