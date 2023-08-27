#pragma once 
#ifndef RADARODOMETRY_H
#define RADARODOMETRY_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include "nuscenes2bag/RadarObject.h"
#include "nuscenes2bag/RadarObjects.h"
#include "msgs_radar/RadarTargetExtended.h"
#include "msgs_radar/RadarScanExtended.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class RadarOdometry
{
public:
    RadarOdometry(){};
    ~RadarOdometry(){};

    PointCloud::Ptr curr_cloud_;

}; // class RadarOdometry

#endif // RADARODOMETRY_H