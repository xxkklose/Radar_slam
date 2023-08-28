#include "radarTypes.h"
using namespace RadarTypes;
using namespace ulit;
bool Preprocess::initParams(ros::NodeHandle& nh){
    if (dataset_type_ == "nuScenes_dataset")
    {
        //TODO : add nuScenes dataset
        sub_radar_ = nh.subscribe<nuscenes2bag::RadarObjects>("/radar_scan", 10,  boost::bind(&Preprocess::radar1Callback, this, _1));
    }else if(dataset_type_ == "4D_Radar_SLAM_dataset")
    {
        sub_radar_ = nh.subscribe<msgs_radar::RadarScanExtended>("/radar_scan", 10,  boost::bind(&Preprocess::radar2Callback, this, _1));
    }else if(dataset_type_ == "mulran")
    {
        //TODO : add mulran dataset
        if(!getRadarFileFormDir(seq_dir_, "png")) 
        {
            ROS_ERROR("radar file empty!");
            return false;
        }
        std::cout << "radar file size: " << radar_files_.size() << std::endl;
    }else
    {
        ROS_ERROR("input dataset type error!");
        return false;
    }

    sub_gps_ = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 10, boost::bind(&Preprocess::gpsCallback, this, _1));
    sub_groundtruth_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, boost::bind(&Preprocess::groundtruthCallback, this, _1));
    pub_gps_path_ = nh.advertise<nav_msgs::Path>("/gps/path", 10);
    pub_groundtruth_path_ = nh.advertise<nav_msgs::Path>("/groundtruth/path", 10);
    pub_radar_ = nh.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud2", 10);

    return true;
}

inline bool Preprocess::exists(const std::string& name) {
    struct stat buffer;
    return !(stat (name.c_str(), &buffer) == 0);
}

bool Preprocess::getRadarFileFormDir(std::string seq_dir, std::string extension)
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
    if(radar_files_.size() == 0) return false;
    return true;
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

bool Preprocess::filterRadarCloud(){
    //根据curr_cloud_和curr_radar_cloud_进行滤波,去除地面点和噪点
    if(!NormalByPCA()){
        ROS_WARN("wait for radar data!");
        return false;
    }
    PointCloud::Ptr ground_cloud = PointCloud::Ptr(new PointCloud);
    //TODO : adjust ego_normal
    Vector3d ego_normal(0, 0, 1);

    for(int i = 0; i < curr_cloud_ptr_->points.size(); i++)
    {
        Vector3d point(curr_cloud_ptr_->points[i].x, curr_cloud_ptr_->points[i].y, curr_cloud_ptr_->points[i].z);
        // if(EuclideanNorm(point) > radius_threshold_ ) continue;
        if(point.z() < - height_threshold_ || point.z() > height_threshold_ ) continue;
    
        Vector3d normal(curr_cloud_normals_ptr_->points[i].normal_x, curr_cloud_normals_ptr_->points[i].normal_y, curr_cloud_normals_ptr_->points[i].normal_z);
        if(normal.dot(ego_normal) < cos_angle_threshold_){
            ground_cloud->points.push_back(curr_cloud_ptr_->points[i]);
        }
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    fitPlaneByRansac(ground_cloud, inliers, coefficients);

    if(processed_cloud_ptr_->points.size() != 0) processed_cloud_ptr_->clear();
    if(processed_radar_cloud_.size() != 0) processed_radar_cloud_.clear();
    processed_cloud_ptr_->points.resize(curr_cloud_ptr_->points.size());

    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];

    for(int i = 0; i < curr_cloud_ptr_->points.size(); i++)
    {
        PointT point = curr_cloud_ptr_->points[i];
        if((A * point.x + B * point.y + C * point.z + D) < 0){
            processed_cloud_ptr_->points.push_back(curr_cloud_ptr_->points[i]);
            processed_radar_cloud_.push_back(curr_radar_cloud_[i]);
        } 
    }
    std::cout << "processed_cloud_ptr_->points.size(): " << processed_cloud_ptr_->points.size() << "\n";
    std::cout << "processed_radar_cloud_.size(): " << processed_radar_cloud_.size() << "\n";

    return true;
}

bool Preprocess::NormalByPCA(){
    if(curr_cloud_ptr_->points.size() == 0) return false;
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(curr_cloud_ptr_);

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(curr_cloud_ptr_);
    ne.setSearchMethod(kdtree);
    ne.setKSearch(20);

    if(curr_cloud_normals_ptr_->points.size() != 0) curr_cloud_normals_ptr_->clear();
    curr_cloud_normals_ptr_->points.resize(curr_cloud_ptr_->points.size());
    ne.compute(*curr_cloud_normals_ptr_);
    return true;
}

void Preprocess::fitPlaneByRansac(const PointCloud::Ptr& input_cloud, 
                                  const pcl::PointIndices::Ptr& inliers,
                                  const pcl::ModelCoefficients::Ptr& coefficients){ 
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
    seg.setMethodType(pcl::SAC_RANSAC);    //设置方法【聚类或随机样本一致性】
    seg.setMaxIterations(100);             //设置最大迭代次数
    seg.setDistanceThreshold(0.1);         //设置内点到模型的距离允许最大值
    seg.setInputCloud(input_cloud);

    seg.segment(*inliers, *coefficients);
}

void Preprocess::radar1Callback(const nuscenes2bag::RadarObjects::ConstPtr& msg){

}

void Preprocess::radar2Callback(const msgs_radar::RadarScanExtended::ConstPtr& msg){
    if(curr_cloud_ptr_->points.size() != 0) curr_cloud_ptr_->clear();
    if(curr_radar_cloud_.size() != 0) curr_radar_cloud_.clear();
    curr_cloud_ptr_->points.resize(msg->targets.size());
    for (int i = 0; i < msg->targets.size(); i++)
    {
        PointT point;
        point.x = msg->targets[i].range * cos(msg->targets[i].elevation) * cos(msg->targets[i].azimuth);
        point.y = msg->targets[i].range * cos(msg->targets[i].elevation) * sin(msg->targets[i].azimuth);
        point.z = - msg->targets[i].range * sin(msg->targets[i].elevation);
        point.intensity = msg->targets[i].snr;
        curr_cloud_ptr_->points.push_back(point);

        RadarObject radar_object(point.x, point.y, point.z, msg->targets[i].velocity, 
            msg->targets[i].power, msg->targets[i].snr, msg->targets[i].detectionconfidence); 
        curr_radar_cloud_.push_back(radar_object);
    }

    sensor_msgs::PointCloud2 radar_cloud_msg;
    pcl::toROSMsg(*curr_cloud_ptr_, radar_cloud_msg);
    radar_cloud_msg.header.frame_id = "odom";
    radar_cloud_msg.header.stamp = msg->header.stamp;
    pub_radar_.publish(radar_cloud_msg);
}

void Preprocess::publishRadarCloud(){
    sensor_msgs::PointCloud2 radar_cloud_msg;
    // pcl::toROSMsg(*curr_cloud_ptr_, radar_cloud_msg);
    pcl::toROSMsg(*processed_cloud_ptr_, radar_cloud_msg);
    radar_cloud_msg.header.frame_id = "odom";
    radar_cloud_msg.header.stamp = ros::Time::now();
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


Preprocess::Preprocess() : point_num_(0)
{
    std::cout << "Preprocess constructor called!" << std::endl;
}

Preprocess::~Preprocess()
{
    std::cout << "Preprocess destructor called!" << std::endl;
}
