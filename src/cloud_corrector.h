#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <cmath>

namespace static_map
{
namespace front_end
{

template <typename PointType>
class CloudCorrector
{
public:
  CloudCorrector() = default;
  ~CloudCorrector() = default;
  
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
  
  
  // refer to paper
  // "LiDAR point clouds correction acquired from a moving car based on CAN-bus 
  // data" 
  // link: https://arxiv.org/pdf/1706.05886.pdf
  void CorrectCloud( PointCloudSourcePtr& cloud
    , double linear_motion, double angle_motion /* rad */)
  {
    if( cloud->empty() )
      return;
    
    // 角速度和线速度都过小的情况下，可以考虑不调整点云
    if( std::fabs( linear_motion ) < 0.01
      && std::fabs( angle_motion ) < 0.2 / 180. * M_PI )
      return;
    
    size_t point_size = cloud->size();
    double start_rad = std::atan2( (*cloud)[0].y, (*cloud)[0].x );
    double end_rad = std::atan2( (*cloud)[point_size-1].y, (*cloud)[point_size-1].x );
    
//     std::cout << "start at " << start_rad << " and end at " << end_rad << std::endl;
    
    double lidar_inner_delta_rad = start_rad + DOUBLE_M_PI - end_rad;
    while(lidar_inner_delta_rad < M_PI ) lidar_inner_delta_rad += DOUBLE_M_PI;
    
//     std::cout << "angle range : " << lidar_inner_delta_rad << std::endl;
    
    // 雷达数据通常来说是不止360°角的，这样我们在考虑内部旋转角的时候需要考虑这个信息
    double max_angle_calculated = -0.1;
    for( size_t i = 0; i < point_size; ++i )
    {
      auto& point = (*cloud)[i];
      Eigen::Vector3d raw_point;
      raw_point << point.x, point.y, point.z;
      
      double alpha = std::atan2( point.y, point.x ) - start_rad;
      while( alpha > 0 ) alpha -= DOUBLE_M_PI;
      
      if( std::fabs( alpha ) > max_angle_calculated - 1e-6 )
        max_angle_calculated = std::fabs( alpha );
      else
        alpha -= DOUBLE_M_PI;
      
      alpha = std::fabs( alpha );
      
      double delta_x_alpha = linear_motion * alpha / lidar_inner_delta_rad;
      double delta_theta_alpha = angle_motion * alpha / lidar_inner_delta_rad;
      double theta_alpha = delta_theta_alpha;
      
      Eigen::Vector3d lidar_location;
      lidar_location << delta_x_alpha * std::cos(0.5*delta_theta_alpha)
        , delta_x_alpha * std::sin(0.5*delta_theta_alpha)
        , 0;
      
      Eigen::Matrix3d transform;
      transform << std::cos(theta_alpha), -std::sin(theta_alpha), 0.
        , std::sin(theta_alpha), std::cos(theta_alpha), 0.
        , 0., 0., 1. ;
      Eigen::Vector3d corrected_point;
      corrected_point = lidar_location + transform * raw_point;
      
      point.x = corrected_point[0];
      point.y = corrected_point[1];
      point.z = corrected_point[2];
    }
  }
};
  
  
} // namepsace front_end
} // namespace static_map