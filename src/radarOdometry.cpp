#include "radarOdometry.h"
#include "preprocess.h"
#include <fstream>

//for radar pre_process class
// std::shared_ptr<pre_process> pre_process_ptr(new pre_process());
Preprocess pre_process;
RadarOdometry radar_odometry;

std::ofstream debug_log;

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radarOdometry");
    ros::NodeHandle nh;
    nh.param<std::string>("dataset_type", pre_process.dataset_type_, "");
    nh.param<std::string>("seq_dir", pre_process.seq_dir_, "");
    ROS_WARN("input dataset type: %s \n",pre_process.dataset_type_.c_str());

    //for debug
    debug_log.open("debug_log.txt", std::ios::app);

    if (pre_process.dataset_type_ == "nuScenes_dataset")
    {
        //TODO : add nuScenes dataset
        pre_process.getRadarFileFormDir(pre_process.seq_dir_, "png");
        std::cout << "radar file size: " << pre_process.radar_files_.size() << std::endl;  
    }else if(pre_process.dataset_type_ == "4D_Radar_SLAM_dataset")
    {
        pre_process.initParams(nh);
    }else if(pre_process.dataset_type_ == "mulran")
    {
        //TODO : add mulran dataset
    }else
    {
        ROS_ERROR("input dataset type error!");
        return -1;
    }

    while(ros::ok())
    {
        ros::spinOnce();
    }

    //for debug
    if(!ros::ok()) debug_log.close();    
    return 0;
}
