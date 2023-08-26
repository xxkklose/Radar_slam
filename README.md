# Radar_SLAM

This is a radar slam project which has so much work to do.

## Dataset 
### Mulran

- Download [mulran dataset](https://sites.google.com/view/mulran-pr/download)
- Use mulran [ROS player](https://github.com/irapkaist/file_player_mulran) to paly dataset

### 4D Radar Dataset 
- SJTU present a novel SLAM dataset containing 4D radar data, LiDAR data and ground truth. It covers different challenging scenes. To the best of our knowledge, there are limited public automotive datasets incorporating 4D radars. [DOWNLOAD](https://robotics.sjtu.edu.cn/upload/file/dataset/4d_radar_dataset/4D_radar_dataset.zip)

- Before use this dataset, you should add an additional message package named [msgs_radar](https://github.com/xxkklose/msgs_radar) to your workspace.

```bash
cd <your_radar_slam_workspace>/src
git clone https://github.com/xxkklose/msgs_radar.git
cd ..
catkin_make
```

## Getting Started

### Prerequisites

#### 1.1 Ubuntu and ROS
Ubuntu >= 20.04

ROS >= noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation).


### Installing
 
#### Step.1

 Clone the repository and catkin_make.
 
```bash
mkdir -p RadarSLAM_ws/src
cd RadarSLAM_ws/src/
git clone https://github.com/xxkklose/Radar_slam.git
cd ..
catkin_make
```

### Run

```bash
roslaunch radar_slam run_radar_slam.launch
```

### TODO
- [ ] 测试实验
- [ ] 速度估计
- [ ] 里程计设计
- [ ] 回环检测
