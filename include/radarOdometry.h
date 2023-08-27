#pragma once 
#ifndef RADARODOMETRY_H
#define RADARODOMETRY_H

#include <ros/ros.h>
#include <vector>
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

#include "preprocess.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class RadarOdometry
{
public:
    RadarOdometry(){};
    ~RadarOdometry(){};

    PointCloud::Ptr curr_cloud_;
    std::vector<double> vel_vector_;
    std::vector<double> power_vector_;
    std::vector<double> snr_vector_;
    std::vector<double> det_confi_vector_;

    double keyframe_delta_trans_;
    double keyframe_delta_angle_;
    double keyframe_delta_time_;
    
    void getDataFromPreprocess(const Preprocess& pre_process, const ros::Time& curr_time);

    void matchScan2Scan(const PointCloud::Ptr& last_cloud, const PointCloud::Ptr& curr_cloud, Eigen::Matrix4f& transform);

}; // class RadarOdometry

#endif // RADARODOMETRY_H