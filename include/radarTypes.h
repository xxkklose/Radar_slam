// #pragma once
#ifndef RADARTYPES_H
#define RADARTYPES_H

#define PCL_NO_PRECOMPILE

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
#include <pcl/registration/ndt.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include "ulit.h"

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

//msc rad4r dataset format
struct RadarPoint
{
    PCL_ADD_POINT4D;  // 添加XYZ坐标
    float alpha;      // alpha信息
    float beta;       // beta信息
    float range;      // range信息
    float doppler;    // doppler信息
    float power;      // power信息
    float recoveredSpeed; // recoveredSpeed信息
    uint16_t dotFlags;  // dotFlags信息
    uint16_t denoiseFlag; // denoiseFlag信息
    uint16_t historyFrameFlag; // historyFrameFlag信息
    uint16_t dopplerCorrectionFlag; // dopplerCorrectionFlag信息
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保对齐
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPoint,
    (float, x, x)                // 定义X坐标字段
    (float, y, y)                // 定义Y坐标字段
    (float, z, z)                // 定义Z坐标字段
    (float, alpha, alpha)        // 定义alpha字段
    (float, beta, beta)          // 定义beta字段
    (float, range, range)        // 定义range字段
    (float, doppler, doppler)    // 定义doppler字段
    (float, power, power)        // 定义power字段
    (float, recoveredSpeed, recoveredSpeed)  // 定义recoveredSpeed字段
    (uint16_t, dotFlags, dotFlags)  // 定义dotFlags字段
    (uint16_t, denoiseFlag, denoiseFlag)  // 定义denoiseFlag字段
    (uint16_t, historyFrameFlag, historyFrameFlag)  // 定义historyFrameFlag字段
    (uint16_t, dopplerCorrectionFlag, dopplerCorrectionFlag)  // 定义dopplerCorrectionFlag字段
)

typedef pcl::PointCloud<RadarPoint> RadarCloud;

/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     
    float roll;         
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

namespace RadarTypes{
    //4D radar dataset format
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

    #if 0 
      PointCloud::Ptr curr_cloud_ptr_;
      std::vector<RadarObject> curr_radar_cloud_;
      //after filtering
      PointCloud::Ptr processed_cloud_ptr_ = PointCloud::Ptr(new PointCloud);
      std::vector<RadarObject> processed_radar_cloud_;
      //normal cloud
      pcl::PointCloud<pcl::Normal>::Ptr curr_cloud_normals_ptr_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    #endif // USE_4D_RADAR_SLAM_DATASET

    RadarCloud::Ptr curr_cloud_ptr_ = RadarCloud::Ptr(new RadarCloud);
    ros::Time curr_cloud_time_;
    RadarCloud::Ptr processed_cloud_ptr_ = RadarCloud::Ptr(new RadarCloud);
    //normal cloud
    pcl::PointCloud<pcl::Normal>::Ptr curr_cloud_normals_ptr_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    //for PCA analysis
    double height_threshold_ = 0.5;
    double cos_angle_threshold_ = 0.8;
    double radius_threshold_;

    int point_num_;
    
    // TODO: use different method (0. visual surpport 1. radar only 2. Deep learning)
    int method_ = 1;

    Preprocess(ros::NodeHandle& nh);
    ~Preprocess();

    /*
    * @brief: init the parameters
    */
    bool initParams();

    bool allocateMemory();

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
    void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void radar2Callback(const msgs_radar::RadarScanExtended::ConstPtr& msg);
    void publishRadarCloud();

private: 

    //ros related
    ros::NodeHandle nh_;
    ros::Subscriber sub_radar_;
    ros::Subscriber sub_gps_;
    ros::Subscriber sub_groundtruth_;
    ros::Publisher pub_radar_;
    ros::Publisher pub_gps_path_;
    ros::Publisher pub_groundtruth_path_;

    std::string pointCloudTopic_;
    std::string gpsTopic_;

    std::string mapFrame_;
    std::string odomFrame_;
    std::string baseFrame_;
    std::string radarFrame_;

    nav_msgs::Path path_; 
    nav_msgs::Path path_groundtruth_;

};


class RadarOdometry
{
public:
    RadarOdometry(ros::NodeHandle& nh);
    ~RadarOdometry(){};

    ros::Subscriber sub_radar_;
    ros::Publisher pub_radar_;
    ros::Publisher pub_path_;
    ros::Publisher pub_odometry_;
    nav_msgs::Path path_msg_;

    std::string mapFrame_;
    std::string odomFrame_;
    std::string baseFrame_;
    std::string radarFrame_;

    double keyframe_delta_trans_;
    double keyframe_delta_angle_;
    double keyframe_delta_time_;

    bool notNew_ = false;

    float transformTobeMapped[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //当前帧的估计位姿
    float lastTransformTobeMapped[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //上一帧的估计位姿 
    std::queue<std::pair<RadarCloud,double>> radar_buffer_;
    double curr_time_;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_ = pcl::PointCloud<PointTypePose>::Ptr(new pcl::PointCloud<PointTypePose>());

    RadarCloud::Ptr map_ = RadarCloud::Ptr(new RadarCloud); 
    RadarCloud::Ptr curr_cloud_ = RadarCloud::Ptr(new RadarCloud);
    RadarCloud::Ptr last_cloud_ = RadarCloud::Ptr(new RadarCloud);
    RadarCloud::Ptr transformed_cloud_ = RadarCloud::Ptr(new RadarCloud);

    pcl::NormalDistributionsTransform<RadarPoint, RadarPoint> ndt_ = pcl::NormalDistributionsTransform<RadarPoint, RadarPoint>();
    bool has_converged_;
    double fitness_score_;

    Eigen::Matrix4f origin_pose_ = Eigen::Matrix4f::Identity();

    void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

    Eigen::Matrix4f matchByNDT();
    void updatePoseAndMap(const Eigen::Matrix4f& diff_pose);
    void transMat4f2XYZRPY(const Eigen::Matrix4f& diff_pose);
    bool saveFrame();
    void publishRadarCloud();
    void publishPath();
    void publishOdometry();

private:


}; // class RadarOdometry

class Pose {
public:
  Pose() { x = y = z = roll = pitch = yaw = 0.0; }
  virtual ~Pose() = default;
  void setPose(const Eigen::Matrix4f &t, const Eigen::Matrix3d &m) {
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    Vector3d euler = m.eulerAngles(2, 1, 0);
    yaw = euler(0);
    pitch = euler(1);
    roll = euler(2);
  }

  double calDistance() {
    double dis = sqrt(x * x + y * y + z * z);
    return dis;
  }
  void operator=(const Pose &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    roll = p.roll;
    pitch = p.pitch;
    yaw = p.yaw;
  }

  Pose operator+(const Pose &p) const {
    Pose res;
    res.x = x + p.x;
    res.y = y + p.y;
    res.z = z + p.z;
    res.roll = roll + p.roll;
    res.pitch = pitch + p.pitch;
    res.yaw = yaw + p.yaw;
    return res;
  }

  Pose operator-(const Pose &p) const {
    Pose res;
    res.x = x - p.x;
    res.y = y - p.y;
    res.z = z - p.z;

    double diff_rad = yaw - p.yaw;
    if (diff_rad >= M_PI)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    res.yaw = diff_rad;

    // TODO ? is necessary ?
    res.roll = 0;
    res.pitch = 0;

    return res;
  }
  friend inline std::ostream &operator<<(std::ostream &os, const Pose &p) {
    os << std::fixed << std::setprecision(2) << " Position: (x:" << p.x
       << ") (y:" << p.y << ") (z:" << p.z << "); "
       << "Rotation: (roll:" << p.roll << ") (pitch:" << p.pitch
       << ") (yaw:" << p.yaw << ")\n";
    return os;
  }

  double x, y, z, roll, pitch, yaw;
};
}// namespace RadarTypes

#endif // RADARTYPES_H