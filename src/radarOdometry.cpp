#include "preprocess.h"
#include <fstream>

//for radar preprocess class
// std::shared_ptr<Preprocess> preprocess_ptr(new Preprocess());
Preprocess preprocess;
std::ofstream debug_log;

ros::Subscriber sub_radar;
ros::Publisher pub_radar;

void radar1Callback(const nuscenes2bag::RadarObjects::ConstPtr& msg){

}


void radar2Callback(const msgs_radar::RadarScanExtended::ConstPtr& msg){
    //for debug
    // if (debug_log.is_open())
    // {
    //     debug_log << "radar point size: " << msg->targets.size() << std::endl;
    //     debug_log << "radar point[0].range: " << msg->targets[0].range << std::endl;
    //     debug_log << "radar point[0].azimuth: " << msg->targets[0].azimuth << std::endl;
    //     debug_log << "radar point[0].elevation: " << msg->targets[0].elevation << std::endl;
    //     debug_log << "radar point[0].velocity: " << msg->targets[0].velocity << std::endl;
    // }   
    PointCloud::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = 0; i < msg->targets.size(); i++)
    {
        PointType point;
        point.x = msg->targets[i].range * cos(msg->targets[i].elevation) * cos(msg->targets[i].azimuth);
        point.y = msg->targets[i].range * cos(msg->targets[i].elevation) * sin(msg->targets[i].azimuth);
        point.z = msg->targets[i].range * sin(msg->targets[i].elevation);
        point.intensity = msg->targets[i].power;
        radar_cloud->points.push_back(point);
    }

    sensor_msgs::PointCloud2 radar_cloud_msg;
    pcl::toROSMsg(*radar_cloud, radar_cloud_msg);
    radar_cloud_msg.header.frame_id = "radar";
    radar_cloud_msg.header.stamp = ros::Time::now();
    pub_radar.publish(radar_cloud_msg);
}

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radarOdometry");
    ros::NodeHandle nh;
    nh.param<std::string>("dataset_type", preprocess.dataset_type_, "");
    nh.param<std::string>("seq_dir", preprocess.seq_dir_, "");
    ROS_WARN("input dataset type: %s \n",preprocess.dataset_type_.c_str());
    pub_radar = nh.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud2", 10);

    //for debug
    debug_log.open("debug_log.txt", std::ios::app);

    if (preprocess.dataset_type_ == "mulran")
    {
        sub_radar = nh.subscribe<nuscenes2bag::RadarObjects>("/radar/points", 10, radar1Callback);
        preprocess.getRadarFileFormDir(preprocess.seq_dir_, "png");
        std::cout << "radar file size: " << preprocess.radar_files_.size() << std::endl;
    }else if(preprocess.dataset_type_ == "4D_Radar_SLAM_dataset")
    {
        sub_radar = nh.subscribe<msgs_radar::RadarScanExtended>("/radar_scan", 10, radar2Callback);
    }else
    {
        std::cout << "Please input the correct data type!" << std::endl;
    }

    while(ros::ok())
    {
        ros::spinOnce();
    }

    //for debug
    if(!ros::ok()) debug_log.close();    
    return 0;
}
