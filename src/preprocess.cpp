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

Preprocess::Preprocess() : point_num_(0)
{
    std::cout << "Preprocess constructor called!" << std::endl;
}

Preprocess::~Preprocess()
{
    std::cout << "Preprocess destructor called!" << std::endl;
}
