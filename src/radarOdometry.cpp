#include "preprocess.h"

#include <fstream>

//for radar preprocess class
// std::shared_ptr<Preprocess> preprocess_ptr(new Preprocess());
Preprocess preprocess;
std::ofstream debug_log;

// ros related
ros::Subscriber sub_radar;
ros::Subscriber sub_gps;
ros::Subscriber sub_groundtruth;
ros::Publisher pub_radar;
ros::Publisher pub_gps_path;
ros::Publisher pub_groundtruth_path;
nav_msgs::Path path; 
nav_msgs::Path path_groundtruth;


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
    radar_cloud_msg.header.frame_id = "odom";
    radar_cloud_msg.header.stamp = msg->header.stamp;
    pub_radar.publish(radar_cloud_msg);
}

void radar3Callback(const nuscenes2bag::RadarObjects::ConstPtr& msg){

}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    geometry_msgs::PoseStamped gps_point;
    gps_point.header.frame_id = "odom";
    gps_point.header.stamp = msg->header.stamp;
    double UTME, UTMN;
    
    if(!preprocess.init_gps_)
    {
        preprocess.init_gps_ = true;
        preprocess.LonLat2UTM(msg->longitude, msg->latitude, UTME, UTMN);
        preprocess.gps_origin_.x() = UTME;
        preprocess.gps_origin_.y() = UTMN;
        preprocess.gps_origin_.z() = msg->altitude;
        gps_point.pose.position.x = 0;
        gps_point.pose.position.y = 0;
        gps_point.pose.position.z = 0;
        path.poses.push_back(gps_point);
        return;
    }

    preprocess.LonLat2UTM(msg->longitude, msg->latitude, UTME, UTMN);
    gps_point.pose.position.x = UTME - preprocess.gps_origin_.x();
    gps_point.pose.position.y = UTMN - preprocess.gps_origin_.y();
    gps_point.pose.position.z = msg->altitude - preprocess.gps_origin_.z();

    path.poses.push_back(gps_point);

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    pub_gps_path.publish(path);
}

void groundtruthCallback(const nav_msgs::Odometry::ConstPtr& msg){

    // std::cout << msg.pose.size() << std::endl;


    std::cout << msg->pose.pose.position.x << '\n';
    std::cout << msg->pose.pose.position.y << '\n';
    std::cout << msg->pose.pose.position.z << '\n';
    std::cout << msg->pose.pose.orientation.x << '\n';
    std::cout << msg->pose.pose.orientation.y << '\n';
    std::cout << msg->pose.pose.orientation.z << '\n';
    std::cout << msg->pose.pose.orientation.w << '\n';

    geometry_msgs::PoseStamped gps_point;

    if(preprocess.init_groundtruth_ == false)
    {
        preprocess.init_groundtruth_ = true;
        preprocess.groundtruth_origin_.x() = msg->pose.pose.position.x;
        preprocess.groundtruth_origin_.y() = msg->pose.pose.position.y;
        preprocess.groundtruth_origin_.z() = msg->pose.pose.position.z;
        gps_point.pose.position.x = 0;
        gps_point.pose.position.y = 0;
        gps_point.pose.position.z = 0;
        gps_point.pose.orientation.x = msg->pose.pose.orientation.x;
        gps_point.pose.orientation.y = msg->pose.pose.orientation.y;
        gps_point.pose.orientation.z = msg->pose.pose.orientation.z;
        gps_point.pose.orientation.w = msg->pose.pose.orientation.w;
        gps_point.header.frame_id = "odom";
        gps_point.header.stamp = msg->header.stamp;
        path_groundtruth.poses.push_back(gps_point);
        return;
    }

    gps_point.pose.position.x = msg->pose.pose.position.x - preprocess.groundtruth_origin_.x();
    gps_point.pose.position.y = msg->pose.pose.position.y - preprocess.groundtruth_origin_.y();
    gps_point.pose.position.z = msg->pose.pose.position.z - preprocess.groundtruth_origin_.z();
    gps_point.pose.orientation.x = msg->pose.pose.orientation.x;
    gps_point.pose.orientation.y = msg->pose.pose.orientation.y;
    gps_point.pose.orientation.z = msg->pose.pose.orientation.z;
    gps_point.pose.orientation.w = msg->pose.pose.orientation.w;
    gps_point.header.frame_id = "odom";
    gps_point.header.stamp = msg->header.stamp;
    path_groundtruth.poses.push_back(gps_point);

    path_groundtruth.header.frame_id = "odom";
    path_groundtruth.header.stamp = ros::Time::now();

    pub_groundtruth_path.publish(path_groundtruth);
}

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radarOdometry");
    ros::NodeHandle nh;
    nh.param<std::string>("dataset_type", preprocess.dataset_type_, "");
    nh.param<std::string>("seq_dir", preprocess.seq_dir_, "");
    ROS_WARN("input dataset type: %s \n",preprocess.dataset_type_.c_str());

    //for debug
    debug_log.open("debug_log.txt", std::ios::app);

    if (preprocess.dataset_type_ == "nuScenes_dataset")
    {
        
    }else if(preprocess.dataset_type_ == "4D_Radar_SLAM_dataset")
    {
        sub_radar = nh.subscribe<msgs_radar::RadarScanExtended>("/radar_scan", 10, radar2Callback);
        sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 10, gpsCallback);
        sub_groundtruth = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, groundtruthCallback);
        pub_gps_path = nh.advertise<nav_msgs::Path>("/gps/path", 10);
        pub_groundtruth_path = nh.advertise<nav_msgs::Path>("/groundtruth/path", 10);
        pub_radar = nh.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud2", 10);
    }else if(preprocess.dataset_type_ == "mulran")
    {
        sub_radar = nh.subscribe<nuscenes2bag::RadarObjects>("/radar/points", 10, radar3Callback);
        preprocess.getRadarFileFormDir(preprocess.seq_dir_, "png");
        std::cout << "radar file size: " << preprocess.radar_files_.size() << std::endl;
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
