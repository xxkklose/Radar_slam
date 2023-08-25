#include "preprocess.h"

//for radar preprocess class
std::shared_ptr<Preprocess> preprocess_ptr(new Preprocess());

void radarPointCloudCallback(const nuscenes2bag::RadarObjects::ConstPtr& input)
{
    PointCloud::Ptr cloud(new PointCloud);
    std::cout << "input->objects.size() = " << input->objects[0].id << std::endl;
}

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radar_slam");
    ros::NodeHandle nh;
    ros::Subscriber subPointCloud = nh.subscribe<nuscenes2bag::RadarObjects>("/radar_front", 100, radarPointCloudCallback);

    ros::spin();
    return 0;
}
