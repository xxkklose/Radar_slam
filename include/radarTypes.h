// #pragma once
#ifndef RADARTYPES_H
#define RADARTYPES_H

#include <string>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include "ulit.h"

// using nuScenes dataset
#include "nuscenes2bag/RadarObject.h"
#include "nuscenes2bag/RadarObjects.h"
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

// using 4D Radar SLAM dataset
#include "msgs_radar/RadarScanExtended.h"
/*
std_msgs/Header header

# Extended targets data of the scan
msgs_radar/RadarTargetExtended[] targets

# The unambiguous interval of the scan
float32 umabiguos_int_range
float32 umabiguos_int_velocity
float32 umabiguos_int_azimuth
float32 umabiguos_int_elevation
*/
#include "msgs_radar/RadarTargetExtended.h"
/*
# Range: Range of the target point in m
float32 range

# Azimuth: Azimuth angle in radian of the targt point
float32 azimuth

# Elevation: Elevation angle in radian of the target point
float32 elevation

# Velocity: Target velocity in m/s
float32 velocity

# SNR: Signal noice ratio (formerly intensity)
float32 snr

float32 power

# Radar cross section of the target
float32 rcs

# Detection confidence of the target
float32 detectionconfidence

# Mean square error (db)
float32 mean_square_error_range
float32 mean_square_error_velocity
float32 mean_square_error_azimuth
float32 mean_square_error_elevation
float32 mean_square_error_subarray
float32 mean_square_error_subarray2nd_best

# Standard deviation
float32 std_dev_range   # 0.25m
float32 std_dev_velocity  # 0.1
float32 std_dev_azimuthAngle
float32 std_dev_elevationAngle
float32 std_dev_rcs
*/

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace RadarTypes{
    struct RadarObject{
        double x;
        double y;
        double z;
        double velocity;
        double power;
        double snr;
        double det_confi;

        RadarObject(double x_, double y_, double z_, double velocity_, double power_, double snr_, double det_confi_):
            x(x_), y(y_), z(z_), velocity(velocity_), power(power_), snr(snr_), det_confi(det_confi_){};
    };

    //using std::vector<RadarObject> as RadarCloud
class Preprocess{
public:
    std::string dataset_type_;
    //for mulran or oxford dataset
    std::string seq_dir_;
    std::vector<std::string> radar_files_;

    //for GPS data
    Vector3d gps_origin_;
    bool init_gps_ = false;

    //for groudtruth data
    Vector3d groundtruth_origin_;
    bool init_groundtruth_ = false;

    PointCloud::Ptr curr_cloud_ptr_ = PointCloud::Ptr(new PointCloud);
    std::vector<RadarObject> curr_radar_cloud_;
    //after filtering
    PointCloud::Ptr processed_cloud_ptr_ = PointCloud::Ptr(new PointCloud);
    std::vector<RadarObject> processed_radar_cloud_;
    //normal cloud
    pcl::PointCloud<pcl::Normal>::Ptr curr_cloud_normals_ptr_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    //for PCA analysis
    double height_threshold_ = 0.5;
    double cos_angle_threshold_ = 0.8;
    double radius_threshold_;

    int point_num_;
    
    // TODO: use different method (0. visual surpport 1. radar only 2. Deep learning)
    int method_ = 1;

    Preprocess();
    ~Preprocess();

    /*
    * @brief: init the parameters
    * @param: nh: the node handle of the ros
    */
    bool initParams(ros::NodeHandle& nh);

    /*
    * @brief: check if the file exists
    */
    inline bool exists(const std::string& name);
    /*
    * @brief: get the radar file from the directory
    * @param: seq_dir: the directory of the radar file
    * @param: extension: the extension of the radar file
    */
    bool getRadarFileFormDir(std::string seq_dir, std::string extension);

    /*
    * @brief: GPS data to UTM
    * @param: longitude: the longitude of the GPS data
    * @param: latitude: the latitude of the GPS data
    * @param: UTME: the UTM easting of the GPS data
    * @param: UTMN: the UTM northing of the GPS data
    */
    void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN);

    bool filterRadarCloud();
    bool NormalByPCA();
    void fitPlaneByRansac(const PointCloud::Ptr& input_cloud, 
                          const pcl::PointIndices::Ptr& inliers,
                          const pcl::ModelCoefficients::Ptr& coefficients);

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void groundtruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void radar1Callback(const nuscenes2bag::RadarObjects::ConstPtr& msg);
    void radar2Callback(const msgs_radar::RadarScanExtended::ConstPtr& msg);
    void publishRadarCloud();

private: 
    ros::Subscriber sub_radar_;
    ros::Subscriber sub_gps_;
    ros::Subscriber sub_groundtruth_;
    ros::Publisher pub_radar_;
    ros::Publisher pub_gps_path_;
    ros::Publisher pub_groundtruth_path_;
    nav_msgs::Path path_; 
    nav_msgs::Path path_groundtruth_;
};


class RadarOdometry
{
public:
    RadarOdometry(){};
    ~RadarOdometry(){};

    PointCloud::Ptr map_cloud_;
    PointCloud::Ptr curr_cloud_;
    std::vector<double> vel_vector_;
    std::vector<double> power_vector_;
    std::vector<double> snr_vector_;
    std::vector<double> det_confi_vector_;

    double keyframe_delta_trans_;
    double keyframe_delta_angle_;
    double keyframe_delta_time_;
    
    void getDataFromPreprocess(const Preprocess& pre_process, const ros::Time& curr_time);

    void matchScan2Scan();

}; // class RadarOdometry

}// namespace RadarTypes

#endif // RADARTYPES_H