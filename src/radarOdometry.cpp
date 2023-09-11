#include "radarTypes.h"
#include <fstream>
using namespace RadarTypes;
using namespace ulit;

#define MAX_BUFFER_SIZE 100

std::ofstream debug_log;

RadarOdometry::RadarOdometry(ros::NodeHandle& nh)
{
    //init parameters
    nh.param<double>("/radar_slam/keyframe_delta_trans", keyframe_delta_trans_, 0.25);
    nh.param<double>("/radar_slam/keyframe_delta_angle", keyframe_delta_angle_, 0.15);
    nh.param<double>("/radar_slam/keyframe_delta_time", keyframe_delta_time_, 1.0);
    nh.param<std::string>("/radar_slam/mapFrame", mapFrame_, "map");
    nh.param<std::string>("/radar_slam/odomFrame", odomFrame_, "odom");
    nh.param<std::string>("/radar_slam/baseFrame", baseFrame_, "base_link");
    nh.param<std::string>("/radar_slam/RadarFrame", radarFrame_, "radar");

    sub_radar_ = nh.subscribe("/preprocess/precessed_pointcloud", 10, &RadarOdometry::radarCallback, this);
    pub_radar_ = nh.advertise<sensor_msgs::PointCloud2>("/radar_odometry/radar_cloud", 1000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/radar_odometry/path", 1000);
    pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/radar_odometry/odometry", 1000);

}

void RadarOdometry::radarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
    if (cloud_in->width == 0 || cloud_in->data.empty())
    {
        notNew_ = true;
        return;
    }else{
        notNew_ = false;
    } 
    RadarCloud::Ptr cloud(new RadarCloud);
    pcl::fromROSMsg(*cloud_in, *cloud);
    double curr_time = cloud_in->header.stamp.toSec();
    std::pair<RadarCloud, double> temp_pair = std::make_pair(*cloud, curr_time);
    if(radar_buffer_.size() >= MAX_BUFFER_SIZE) radar_buffer_.pop();
    radar_buffer_.push(temp_pair);
}

Eigen::Matrix4f RadarOdometry::matchByNDT(){
    static bool is_first_frame = true;
    if(is_first_frame){
        is_first_frame = false;
        auto first_frame = radar_buffer_.front();
        *map_ += first_frame.first;
        ndt_.setTransformationEpsilon(0.01);
        ndt_.setStepSize(0.1);
        ndt_.setResolution(1.0); 
        ndt_.setMaximumIterations(35);
        RadarCloud::Ptr first_frame_map(new RadarCloud(*map_));
        // *first_frame_map += *map;
        ndt_.setInputTarget(first_frame_map);
        return Eigen::Matrix4f::Identity();
    }

    Pose init_pose;//TODO : use imu as init pose
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // ndt_.setInputTarget(curr_cloud_);
    // ndt_.setInputSource(last_cloud_);    
    ndt_.setInputTarget(last_cloud_);
    ndt_.setInputSource(curr_cloud_);
    RadarCloud::Ptr output_cloud(new RadarCloud);
    ndt_.align(*output_cloud, init_guess);

    fitness_score_ = ndt_.getFitnessScore();
    Eigen::Matrix4f t_localizer = ndt_.getFinalTransformation();
    has_converged_ = ndt_.hasConverged();
      
    return t_localizer; 
}
void RadarOdometry::updatePoseAndMap(const Eigen::Matrix4f& diff_pose)
{
    // TODO: 使用关键帧
    pcl::transformPointCloud(*curr_cloud_, *transformed_cloud_, diff_pose);

    *last_cloud_ = *transformed_cloud_;

    transMat4f2XYZRPY(diff_pose);
    for(int i = 0; i < 6; i++) lastTransformTobeMapped[i] = transformTobeMapped[i];

    // if(!saveFrame(diff_pose)) return;
   
    // *map_ += *transformed_cloud_;

    return;
}

void RadarOdometry::transMat4f2XYZRPY(const Eigen::Matrix4f& diff_pose)
{
    Eigen::Vector3f translation = diff_pose.block<3, 1>(0, 3);
    transformTobeMapped[0] = translation(0) + lastTransformTobeMapped[0];
    transformTobeMapped[1] = translation(1) + lastTransformTobeMapped[1];
    transformTobeMapped[2] = translation(2) + lastTransformTobeMapped[2];

    // 计算旋转矩阵
    Eigen::Matrix3f rotationMatrix = diff_pose.block<3, 3>(0, 0);

    // 从旋转矩阵获取旋转角 (roll, pitch, yaw)
    Eigen::Vector3f eulerAngle = rotationMatrix.eulerAngles(0, 1, 2);
    transformTobeMapped[3] = eulerAngle[0] + lastTransformTobeMapped[3];
    transformTobeMapped[4] = eulerAngle[1] + lastTransformTobeMapped[4];
    transformTobeMapped[5] = eulerAngle[2] + lastTransformTobeMapped[5]; 
}

bool RadarOdometry::saveFrame()
{
    if(cloudKeyPoses6D_->points.empty()){
        // cloudKeyPoses6D_->push_back(pose6D_);
        return false;
    }
}

void RadarOdometry::publishRadarCloud()
{
    sensor_msgs::PointCloud2 radar_cloud_msg;
    pcl::toROSMsg(*transformed_cloud_, radar_cloud_msg);
    radar_cloud_msg.header.frame_id = mapFrame_;
    radar_cloud_msg.header.stamp = ros::Time::now();
    pub_radar_.publish(radar_cloud_msg);
    return;
}

void RadarOdometry::publishPath()
{

    ros::Time time_obj = ros::Time().fromSec(curr_time_);
    path_msg_.header.frame_id = mapFrame_;
    path_msg_.header.stamp = time_obj;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = mapFrame_;
    pose_stamped.header.stamp = time_obj;
    pose_stamped.pose.position.x = transformTobeMapped[0];
    pose_stamped.pose.position.y = transformTobeMapped[1];
    pose_stamped.pose.position.z = transformTobeMapped[2];
    Vector3f eulerAngle = Vector3f(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
    Quaternionf q;
    q = AngleAxisf(eulerAngle[2], Vector3f::UnitZ()) *
        AngleAxisf(eulerAngle[1], Vector3f::UnitY()) *
        AngleAxisf(eulerAngle[0], Vector3f::UnitX());
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    path_msg_.poses.push_back(pose_stamped);
    pub_path_.publish(path_msg_);
}

void RadarOdometry::publishOdometry()
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = mapFrame_;
    odom_msg.child_frame_id = baseFrame_;
    odom_msg.header.stamp = ros::Time().fromSec(curr_time_);
    odom_msg.pose.pose.position.x = transformTobeMapped[0];
    odom_msg.pose.pose.position.y = transformTobeMapped[1];
    odom_msg.pose.pose.position.z = transformTobeMapped[2];
    Vector3f eulerAngle = Vector3f(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
    Quaternionf q;
    q = AngleAxisf(eulerAngle[2], Vector3f::UnitZ()) *
        AngleAxisf(eulerAngle[1], Vector3f::UnitY()) *
        AngleAxisf(eulerAngle[0], Vector3f::UnitX());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    pub_odometry_.publish(odom_msg);
}


int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radarOdometry");
    ros::NodeHandle nh("~");
    debug_log.open("debug_log.txt", std::ios::app);  //for debug

    RadarOdometry radar_odometry = RadarOdometry(nh);

    ROS_INFO("\033[1;32m----> Radar Odometry Started.\033[0m");

    while(ros::ok())
    {
        ros::spinOnce();
        if(radar_odometry.radar_buffer_.size() == 0) continue;
        if(radar_odometry.notNew_) continue;
        radar_odometry.curr_cloud_ = radar_odometry.radar_buffer_.back().first.makeShared();
        radar_odometry.curr_time_ = radar_odometry.radar_buffer_.back().second;
        Eigen::Matrix4f curr_diff_pose = radar_odometry.matchByNDT();
        radar_odometry.updatePoseAndMap(curr_diff_pose);
        radar_odometry.publishRadarCloud();
        radar_odometry.publishPath();
    }

    //for debug
    if(!ros::ok()) debug_log.close();    
    return 0;
}

