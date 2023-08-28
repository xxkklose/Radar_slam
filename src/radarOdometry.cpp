#include "radarTypes.h"
#include <fstream>
using namespace RadarTypes;
void RadarOdometry::getDataFromPreprocess(const Preprocess& pre_process, const ros::Time& curr_time){
    // curr_cloud_ = pre_process.processed_cloud_;

}

Preprocess pre_process;
RadarOdometry radar_odometry;

std::ofstream debug_log;

void RadarOdometry::matchScan2Scan(){
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //add icp match
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(map_cloud_);
    icp.setInputTarget(curr_cloud_);
    icp.align(*map_cloud_);
    transform = icp.getFinalTransformation();
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
}


int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radarOdometry");
    ros::NodeHandle nh;
    debug_log.open("debug_log.txt", std::ios::app);
    nh.param<std::string>("dataset_type", pre_process.dataset_type_, "");
    nh.param<std::string>("seq_dir", pre_process.seq_dir_, "");
    nh.param<double>("keyframe_delta_trans", radar_odometry.keyframe_delta_trans_, 0.25);
    nh.param<double>("keyframe_delta_angle", radar_odometry.keyframe_delta_angle_, 0.15);
    nh.param<double>("keyframe_delta_time", radar_odometry.keyframe_delta_time_, 1.0);

    ROS_WARN("input dataset type: %s \n",pre_process.dataset_type_.c_str());

    if(!pre_process.initParams(nh)) ROS_ERROR("init params fail!");

    while(ros::ok())
    {
        ros::spinOnce();

        if(!pre_process.filterRadarCloud()) continue;
        // pre_process.publishRadarCloud();

        radar_odometry.getDataFromPreprocess(pre_process, ros::Time::now());
    }

    //for debug
    if(!ros::ok()) debug_log.close();    
    return 0;
}

