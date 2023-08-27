#include "preprocess.h"
inline bool Preprocess::exists(const std::string& name) {
    struct stat buffer;
    return !(stat (name.c_str(), &buffer) == 0);
}

void Preprocess::getRadarFileFormDir(std::string seq_dir, std::string extension)
{
    DIR *dir = opendir(seq_dir.c_str());
    struct dirent *dp;
    while ((dp = readdir(dir)) != NULL) {
        if (exists(dp->d_name)) {
            if (!extension.empty()) {
                std::vector<std::string> parts;
                boost::split(parts, dp->d_name, boost::is_any_of("."));
                if (parts[parts.size() - 1].compare(extension) != 0)
                    continue;
            }
            radar_files_.push_back(dp->d_name);
        }
    }
}

// GPS data to UTM
void Preprocess::LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN)
{
	double lat = latitude;
	double lon = longitude;

	double kD2R = M_PI / 180.0;
	double ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
	double L0 = ZoneNumber * 3.0;

	double a = 6378137.0;
	double F = 298.257223563;
	double f = 1 / F;
	double b = a * (1 - f);
	double ee = (a * a - b * b) / (a * a);
	double e2 = (a * a - b * b) / (b * b);
	double n = (a - b) / (a + b); 
	double n2 = (n * n); 
	double n3 = (n2 * n); 
	double n4 = (n2 * n2); 
	double n5 = (n4 * n);
	double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
	double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
	double gm = 15 * n2 / 16 - 15 * n4 / 32;
	double dt = -35 * n3 / 48 + 105 * n5 / 256;
	double ep = 315 * n4 / 512;
	double B = lat * kD2R;
	double L = lon * kD2R;
	L0 = L0 * kD2R;
	double l = L - L0; 
	double cl = (cos(B) * l); 
	double cl2 = (cl * cl); 
	double cl3 = (cl2 * cl); 
	double cl4 = (cl2 * cl2); 
	double cl5 = (cl4 * cl); 
	double cl6 = (cl5 * cl); 
	double cl7 = (cl6 * cl); 
	double cl8 = (cl4 * cl4);
	double lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
	double t = tan(B); 
	double t2 = (t * t); 
	double t4 = (t2 * t2); 
	double t6 = (t4 * t2);
	double Nn = a / sqrt(1 - ee * sin(B) * sin(B));
	double yt = e2 * cos(B) * cos(B);
	double N = lB;
	N = N + t * Nn * cl2 / 2;
	N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
	N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
	N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
	double E = Nn * cl;
	E = E + Nn * cl3 * (1 - t2 + yt) / 6;
	E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
	E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
	E = E + 500000;
	N = 0.9996 * N;
	E = 0.9996 * (E - 500000.0) + 500000.0;

	UTME = E;
	UTMN = N;
}

void Preprocess::radar1Callback(const nuscenes2bag::RadarObjects::ConstPtr& msg){

}

void Preprocess::radar2Callback(const msgs_radar::RadarScanExtended::ConstPtr& msg){
    PointCloud::Ptr radar_cloud(new PointCloud());

    for (int i = 0; i < msg->targets.size(); i++)
    {
        PointT point;
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
    pub_radar_.publish(radar_cloud_msg);
}


void Preprocess::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    geometry_msgs::PoseStamped gps_point;
    gps_point.header.frame_id = "odom";
    gps_point.header.stamp = msg->header.stamp;
    double UTME, UTMN;
    
    if(!init_gps_)
    {
        init_gps_ = true;
        LonLat2UTM(msg->longitude, msg->latitude, UTME, UTMN);
        gps_origin_.x() = UTME;
        gps_origin_.y() = UTMN;
        gps_origin_.z() = msg->altitude;
        gps_point.pose.position.x = 0;
        gps_point.pose.position.y = 0;
        gps_point.pose.position.z = 0;
        path_.poses.push_back(gps_point);
        return;
    }

    LonLat2UTM(msg->longitude, msg->latitude, UTME, UTMN);
    gps_point.pose.position.x = UTME - gps_origin_.x();
    gps_point.pose.position.y = UTMN - gps_origin_.y();
    gps_point.pose.position.z = msg->altitude - gps_origin_.z();

    path_.poses.push_back(gps_point);

    path_.header.frame_id = "odom";
    path_.header.stamp = ros::Time::now();

    pub_gps_path_.publish(path_);
}

void Preprocess::groundtruthCallback(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::PoseStamped gps_point;
    if(init_groundtruth_ == false)
    {
        init_groundtruth_ = true;
        groundtruth_origin_.x() = msg->pose.pose.position.x;
        groundtruth_origin_.y() = msg->pose.pose.position.y;
        groundtruth_origin_.z() = msg->pose.pose.position.z;
        gps_point.pose.position.x = 0;
        gps_point.pose.position.y = 0;
        gps_point.pose.position.z = 0;
        gps_point.pose.orientation.x = msg->pose.pose.orientation.x;
        gps_point.pose.orientation.y = msg->pose.pose.orientation.y;
        gps_point.pose.orientation.z = msg->pose.pose.orientation.z;
        gps_point.pose.orientation.w = msg->pose.pose.orientation.w;
        gps_point.header.frame_id = "odom";
        gps_point.header.stamp = msg->header.stamp;
        path_groundtruth_.poses.push_back(gps_point);
        return;
    }

    gps_point.pose.position.x = msg->pose.pose.position.x - groundtruth_origin_.x();
    gps_point.pose.position.y = msg->pose.pose.position.y - groundtruth_origin_.y();
    gps_point.pose.position.z = msg->pose.pose.position.z - groundtruth_origin_.z();
    gps_point.pose.orientation.x = msg->pose.pose.orientation.x;
    gps_point.pose.orientation.y = msg->pose.pose.orientation.y;
    gps_point.pose.orientation.z = msg->pose.pose.orientation.z;
    gps_point.pose.orientation.w = msg->pose.pose.orientation.w;
    gps_point.header.frame_id = "odom";
    gps_point.header.stamp = msg->header.stamp;
    path_groundtruth_.poses.push_back(gps_point);

    path_groundtruth_.header.frame_id = "odom";
    path_groundtruth_.header.stamp = ros::Time::now();

    pub_groundtruth_path_.publish(path_groundtruth_);
}

void Preprocess::initParams(ros::NodeHandle& nh){
    sub_radar_ = nh.subscribe<msgs_radar::RadarScanExtended>("/radar_scan", 10,  boost::bind(&Preprocess::radar2Callback, this, _1));
    sub_gps_ = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 10, boost::bind(&Preprocess::gpsCallback, this, _1));
    sub_groundtruth_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, boost::bind(&Preprocess::groundtruthCallback, this, _1));
    pub_gps_path_ = nh.advertise<nav_msgs::Path>("/gps/path", 10);
    pub_groundtruth_path_ = nh.advertise<nav_msgs::Path>("/groundtruth/path", 10);
    pub_radar_ = nh.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud2", 10);
}

Preprocess::Preprocess() : point_num_(0)
{
    std::cout << "Preprocess constructor called!" << std::endl;
}

Preprocess::~Preprocess()
{
    std::cout << "Preprocess destructor called!" << std::endl;
}
