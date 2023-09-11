cmake_minimum_required(VERSION 3.10)
project(radar_slam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

# add_subdirectory(include/jsk_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    msgs_radar
    message_generation
    # nuscenes2bag
    tf2_ros
    cv_bridge
    pcl_conversions
    pcl_ros
    geometry_msgs
    laser_geometry
    sensor_msgs
    # jsk_recognition_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)


message(Eigen: ${EIGEN3_INCLUDE_DIRS})

catkin_package(
    # CATKIN_DEPENDS roscpp rospy std_msgs message_generation nuscenes2bag tf2_ros
    CATKIN_DEPENDS roscpp rospy std_msgs message_generation tf2_ros 
    DEPENDS EIGEN3 PCL OpenCV Boost GTSAM
    INCLUDE_DIRS
)

add_executable(preprocess src/preprocess.cpp)
target_link_libraries(preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(radarOdometry src/radarOdometry.cpp)
target_link_libraries(radarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# add_executable(demo src/demo.cpp)
# target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


