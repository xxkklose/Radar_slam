// #include "ground_segment.h"
​
namespace  front_lidar {
​
/* @brief:地面检测Demo
 * @param [in]: NONE
 * @param [out]: NONE
 * @return NONE
 */
void groundSegmentDemo(void)
{
#if 0
  string cloud_file_str;//读取点云文件路径和文件名
  string config_file_str;//配置文件路径和文件名
​
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);//读入的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);//地面点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);//障碍物点云
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZI>(cloud_file_str, *cloud_ptr);//读取pcd文件，用指针传递给cloud
​
  //实例化地面分割类
  GroundSegment ground_segment(config_file_str);
​
  //地面分割处理,并计时显示
  pcl::StopWatch object_segmenter_timer;
  ground_segment.segment(*cloud_ptr, ground_cloud_ptr, no_ground_cloud_ptr);
  std::cout<<"Ground segment time:"<<object_segmenter_timer.getTime()<<"ms"<<std::endl;
#endif
}
​
/* @brief:构造函数,初始化参数
 * @param [in]: config_file-配置文件路径
 * @param [out]: NONE
 * @return NONE
 */
GroundSegment::GroundSegment(string config_file)
{
​
  //从配置文件读取参数
  config_parser_ptr_.reset(new ConfigParser(config_file));//根据配置文件路径,获取配置文件参数
​
  sensor_height_ = config_parser_ptr_->getDouble("height");                             //传感器安装高低 
  single_height_threshold_ = config_parser_ptr_->getDouble("single_height_threshold");  //与前前个栅格的角度阈值
  local_max_slope_ = config_parser_ptr_->getDouble("local_max_slope");                  //与前一个栅格的角度阈值
  general_min_slope_ = config_parser_ptr_->getDouble("general_min_slope");              //与零点的最小角度阈值
  general_max_slope_ = config_parser_ptr_->getDouble("general_max_slope");              //与零点的最大角度阈值
  ground_max_slope_ = config_parser_ptr_->getDouble("ground_max_slope");                //与地面点的角度阈值
  cell_height_threshold_ = config_parser_ptr_->getDouble("cell_height_threshold");           //栅格内高度差阈值
}
​
/* @brief:析构函数
* @param [in]: NONE
* @param [out]: NONE
* @return NONE
*/
GroundSegment::~GroundSegment()
{
 
}
​
/* @brief:分割线程
* @param [in]: in_cloud-坐标转换后的点云
* @param [out]: object_index_pt-输出结果
* @return NONE
*/
void GroundSegment::segmentThread(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, 
                                  std::vector<int>* object_index_ptr)
{
  // pcl::StopWatch loop_timer;
  std::vector<std::vector<PointXYZRI> > clouds;
​
  pcl::PointXYZI zero_point;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.z = -sensor_height_;
​
  sectorProjection(in_cloud, clouds);
  classify(clouds, object_index_ptr, zero_point);
​
  // std::cout<<"loop time:"<<loop_timer.getTime()<<"ms"<<std::endl;
​
}
​
/* @brief:栅格化投影
  * @param [in]: in_cloud-坐标转换后的点云
  * @param [out]: cloud_vec-栅格化后的点云数据
  * @return NONE
  */
void GroundSegment::sectorProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, 
                                     std::vector<std::vector<PointXYZRI> > &cloud_vec)
{      
  cloud_vec.clear();
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn = *in_cloud;
  //初始化栅格
  cloud_vec.resize(int(ANGLE_NUM));
  for(int i = 0; i < cloud_vec.size(); i++)
  {
    cloud_vec[i].resize(int(RADIUS_NUM));
  }
  //点云投影
  for(int i = 0; i < laserCloudIn.points.size(); i++)
  {
    float x = laserCloudIn.points[i].x;
    float y = laserCloudIn.points[i].y;
    float z = laserCloudIn.points[i].z;
​
    //超出索引的删除
    if(x < -20 || x > 20 || y < 0 || y > 50)
      continue;
    if(x < 1 && x > -1 && y < 1 && y > -1)
      continue;
    
    //距离坐标
    float radius = sqrt(x*x+y*y);
    int radius_loc = int(radius/RADIUS_RES);
    //角度坐标
    float angle = atan2(y,x)*180/M_PI;
    if(angle < 0)
      angle += 360;
    int loc = int(angle/ANGLE_RES);
​
    cloud_vec[loc][radius_loc].points.push_back(laserCloudIn.points[i]);
    cloud_vec[loc][radius_loc].indexs.push_back(i);
    cloud_vec[loc][radius_loc].radius = (radius_loc + 0.5)*RADIUS_RES;
  }
}
​
/* @brief:地面点分割
* @param [in]: cloud_vec-栅格化后的点云数据  zero_point-坐标转换后新地面原点
* @param [out]: object_index_ptr-地面点标记索引
* @return NONE
*/
void GroundSegment::classify(std::vector<std::vector<PointXYZRI> > &cloud_vec,
                             std::vector<int>* object_index_ptr,
                             pcl::PointXYZI zero_point)
{
  float ground = zero_point.z;
  //根据原点进行地面角度校正
  double ground_angle = fabs(ground+sensor_height_)<0.1? 0:atan2(ground + sensor_height_, sqrtf(zero_point.x*zero_point.x + zero_point.y*zero_point.y));
  
  for(int i = 0; i < cloud_vec.size(); i++)
  {
    float prev_radius = 0.f;
    float prev_height = ground;
    float prev_angle = 0.f;
    float prev_distance = 0.f;
    float ground_radius = 0.f;
    float ground_height = ground;
    float i_ground_angle = atan(sin(DEG2RAD(i))*tan(ground_angle));//地面校正偏差，与角度栅格编号有关
    bool prev_ground = true;
    bool current_ground = true;
​
    for (int j = 0; j < cloud_vec[i].size(); j++) //loop through each point in the radial div
    {
      if(cloud_vec[i][j].points.size() > 0)
      { 
        //高度均值
        float sum = 0, min = 0, max = -10;
        for(int k = 0; k < cloud_vec[i][j].points.size(); k++)
        {
          sum += cloud_vec[i][j].points[k].z;
          if(cloud_vec[i][j].points[k].z > max) max = cloud_vec[i][j].points[k].z;
          if(cloud_vec[i][j].points[k].z < min) min = cloud_vec[i][j].points[k].z;
        }
        float current_height = sum/cloud_vec[i][j].points.size();
        float points_distance = cloud_vec[i][j].radius - prev_radius;
        //与前一个栅格的高度差
        float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
        bool local = current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold);
        //与坐标原点的高度差
        float general_height_min_threshold = tan(DEG2RAD(general_min_slope_) - i_ground_angle) * cloud_vec[i][j].radius;
        bool general_min = current_height <= (ground + general_height_min_threshold) && current_height >= (ground - general_height_min_threshold);
        float general_height_max_threshold = tan(DEG2RAD(general_max_slope_) - i_ground_angle) * cloud_vec[i][j].radius;
        bool general_max = current_height <= (ground + general_height_max_threshold) && current_height >= (ground - general_height_max_threshold);
        //与前前个栅格的高度差
        float current_angle = atan2((current_height - prev_height), points_distance);
        bool single = atan2((prev_distance*tan(prev_angle) + points_distance*tan(current_angle)), (prev_distance + points_distance))  > single_height_threshold_;
        //与地面栅格的高度差
        float ground_height_threshold = tan(DEG2RAD(ground_max_slope_) - i_ground_angle) * (cloud_vec[i][j].radius - ground_radius);
        bool height = fabs(current_height - ground_height) > ground_height_threshold;
        //栅格内部高度差判断
        bool cell_in = (max - min) > cell_height_threshold_;
​
​
        //地面栅格判断
        if(general_min)
        {
          current_ground = true;
        }
        else
        {
          if(!general_max)
          {
            current_ground = false;
            if(cell_in)
            {
              for(int l = 0; l< cloud_vec[i][j].indexs.size(); l++)
              {
                if(cloud_vec[i][j].points[l].z < (ground_height + single_height_threshold_))
                {
                  std::swap(cloud_vec[i][j].indexs[l], cloud_vec[i][j].indexs.back());
                  cloud_vec[i][j].indexs.pop_back();
                }
              }
            }
          }
          else
          {
            if(height && single)
            {
              current_ground = false;
              if(cell_in)
              {
                for(int l = 0; l< cloud_vec[i][j].indexs.size(); l++)
                {
                  if(cloud_vec[i][j].points[l].z < (ground_height + single_height_threshold_))
                  {
                    std::swap(cloud_vec[i][j].indexs[l], cloud_vec[i][j].indexs.back());
                            cloud_vec[i][j].indexs.pop_back();
                  }
                }
              }
            }
            else
            {
              // if(!prev_ground && !local  && single)
              // {
              //   current_ground = false;
              // }
              // else
              // {
                current_ground = true;
              // }
            }
          }
        }
        //当前栅格判断结束
        if (current_ground)
        {
          prev_ground = true;
          ground_height = prev_height;
          ground_radius = prev_radius;
        }
        else
        {
          for(int l = 0; l< cloud_vec[i][j].indexs.size(); l++)
            object_index_ptr->at(cloud_vec[i][j].indexs[l]) = 1;
          prev_ground = false;
        }
​
        prev_radius = cloud_vec[i][j].radius;
        prev_height = current_height;
        prev_angle = current_angle;
        prev_distance = points_distance;
      }
    }   
  }
}
​
/* @brief:地面点分割处理
 * @param [in]: in_cloud-预处理后的点云  in_plan_cloud-路径点
 * @param [out]: ground_cloud_ptr-检测到的地面点云  object_cloud_ptr-检测到的目标点云
 * @return true-分割正常  false-分割异常
 */
bool GroundSegment::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, \
                            const pcl::PointCloud<pcl::PointXYZ> &in_plan_cloud, \
                            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr, \
                            pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud_ptr)
{
  //清空输出点云
  ground_cloud_ptr->clear();
  object_cloud_ptr->clear();
​
  std::vector<int> sobject_index;
  sobject_index.clear();
​
  //如果输入点云为空,则直接返回
  if(0 == in_cloud->points.size())
  {
    std::cout<<"In GroundSegment: In cloud size is 0!"<<std::endl;
    return false;
  }
​
  //如果输入路径点为空,则默认路径点为正前方
  way_cloud_ = in_plan_cloud;
  if(0 == in_plan_cloud.points.size())
  {
    std::cout<<"In GroundSegment: In plan cloud size is 0!"<<std::endl;
    
    pcl::PointXYZ point;
    point.x = 0;
    point.z = 0;
    for(int i = 0; i < 50; i++)
    {
      point.y = i;
      way_cloud_.points.push_back(point);
    }
  }
​
  //参数初始化
  raw_cloud_ptr_ = in_cloud;
  object_index.clear();
  object_index.resize(raw_cloud_ptr_->size(), 0);
​
​
  segmentThread(raw_cloud_ptr_, &object_index);
​
  for(int i = 0; i < object_index.size(); i++)
  {
    if(object_index[i] == 1)
      sobject_index.push_back(i);
  }
  
  //地面点分割
  pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
  extract_ground.setInputCloud(raw_cloud_ptr_);
  extract_ground.setIndices(boost::make_shared<std::vector<int> >(sobject_index));
  extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
  extract_ground.filter(*object_cloud_ptr);
  extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
  extract_ground.filter(*ground_cloud_ptr);
}
​
}//namespace  front_lidar