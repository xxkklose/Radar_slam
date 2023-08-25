# Radar_SLAM

This is a radar slam project which has so much work to do.

## Dataset 
### Mulran

- Download [mulran dataset](https://sites.google.com/view/mulran-pr/download)
- Use mulran [ROS player](https://github.com/irapkaist/file_player_mulran) to paly dataset

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

### TODO
- [] 测试实验
- [] 速度估计
- [] 里程计设计
- [] 回环检测