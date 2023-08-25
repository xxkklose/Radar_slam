#pragma once
#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <string>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

// using nuScenes dataset
// #include "nuscenes2bag/RadarObject.h"
// #include "nuscenes2bag/RadarObjects.h"
/*
geometry_msgs/Vector3 pose：物体的位置信息，通常包括X、Y和Z坐标。

uint8 dyn_prop：物体的动态属性，用于描述物体的运动状态（例如，静止、运动、加速等）。

uint16 id：物体的唯一标识符，用于在不同帧中跟踪相同的物体。

float32 rcs：物体的雷达反射截面，用于估计物体的大小或反射强度。

float32 vx 和 float32 vy：物体的X和Y方向上的速度。

float32 vx_comp 和 float32 vy_comp：物体的X和Y方向上的速度的组件。

uint8 is_quality_valid：一个指示质量信息是否有效的标志。

uint8 ambig_state：物体的模糊状态，通常用于处理多解模糊。

uint8 x_rms 和 uint8 y_rms：X和Y方向上的速度估计的均方根误差。

uint8 invalid_state：物体的状态，通常用于指示物体是否有效。

uint8 pdh0：用于特定应用的其他信息。
*/


typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Preprocess{
public:
    //for mulran or oxford dataset
    std::string seq_dir_;
    std::vector<std::string> radar_files_;

    PointCloud input_cloud_;
    int point_num_;
    
    // TODO: use different method (0. visual surpport 1. radar only 2. Deep learning)
    int method_ = 1;

    Preprocess();
    ~Preprocess();

    /*
    * @brief: check if the file exists
    */
    inline bool exists(const std::string& name);
    /*
    * @brief: get the radar file from the directory
    * @param: seq_dir: the directory of the radar file
    * @param: extension: the extension of the radar file
    */
    void getRadarFileFormDir(std::string seq_dir, std::string extension);

};





#endif