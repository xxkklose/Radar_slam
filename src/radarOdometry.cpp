#include "preprocess.h"

//for radar preprocess class
// std::shared_ptr<Preprocess> preprocess_ptr(new Preprocess());
Preprocess preprocess;

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "radarOdometry");
    ros::NodeHandle nh;
    nh.param<std::string>("seq_dir", preprocess.seq_dir_, "");

    preprocess.getRadarFileFormDir(preprocess.seq_dir_, "png");

    std::cout << "radar file size: " << preprocess.radar_files_.size() << std::endl;

    ros::spin();
    return 0;
}
