#pragma once

#include <cmath>
#include <algorithm>

namespace static_map{
namespace back_end{

template <typename PointType>
void M2dp<PointType>::preProcess( 
  const M2dp<PointType>::PointCloudSourcePtr& source )
{
  // remove the shift and rotation
  pcl::PCA<PointType> pca;
  pca.setInputCloud( source );
  Eigen::Matrix3f& vectors = pca.getEigenVectors();
  Eigen::Vector4f& mean = pca.getMean();
  mean_ << mean[0], mean[1], mean[2];
  Eigen::Matrix4f transform;
  transform.block(0,0,3,3) << vectors;
  transform.block(0,3,4,1) << mean;
  transform.block(3,0,1,3) << 0,0,0;

  transform = transform.inverse().eval();
  PointCloudSourcePtr tmp_inner_cloud( new PointCloudSource );
  pcl::transformPointCloud( *source, *tmp_inner_cloud, transform );
  
  for( auto& point: tmp_inner_cloud->points )
  {
    double d = getLength( point );
    if( d <= max_distance_ )
      inner_cloud_->push_back(point);
  }
  
  if( r_ < 1.e-6 )
  {
    PCL_ERROR("r is too small.\n");
    return;
  }
  l_ = std::ceil(std::sqrt( max_distance_ / r_ ));
  A_.resize( p_*q_, l_*t_);
  
  std::cout << "max_distance_ = " << max_distance_ << "  l_ = " << l_ << std::endl;
}

// template <typename PointType>
// Eigen::VectorXi M2dp<PointType>::singleViewProcess( double theta, double phi )
// {
//   // normal
//   Eigen::Vector3f m;
//   m << std::cos(theta) * std::cos(phi), std::cos(theta) * std::sin(phi)
//     , std::sin(theta);
//     
//   PointCloudSourcePtr projected_cloud( new PointCloudSource );
//   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//   coefficients->values.resize(4);
//   coefficients->values[0] = m[0];
//   coefficients->values[1] = m[1];
//   coefficients->values[2] = m[2];
//   coefficients->values[3] = 0.;
// 
//   // Create the filtering object
//   pcl::ProjectInliers<PointType> proj;
//   proj.setModelType( pcl::SACMODEL_PLANE );
//   proj.setInputCloud( inner_cloud_ );
//   proj.setModelCoefficients(coefficients);
//   proj.filter(*projected_cloud);
//   
//   // create the polar PolarCoordinate
//   PointCloudSourcePtr x_axis( new PointCloudSource );
//   x_axis->push_back( PointType(10.,0.,0.) );
//   PointCloudSourcePtr x_axis_projectory( new PointCloudSource );
//   proj.setInputCloud( x_axis );
//   proj.filter( *x_axis_projectory );
//   
//   std::function<PolarCoordinate(PointType&, PointType&)> cal_polar = 
//     [ this ](PointType& axis, PointType& target)
//     {
//       PolarCoordinate coord;
//       coord.length = getLength(target);
//       double cos_angle = (axis.x*target.x + axis.y*target.y + axis.z*target.z) / 
//         ( getLength(axis) * coord.length );
//       
//       // 防止计算精度造成的问题，定义域错误会造成算出的结果为 nan
//       if( cos_angle > 1.0 ) 
//         cos_angle = 1.0;  
//       else if ( cos_angle < -1. ) 
//         cos_angle = -1.;
//       
//       coord.angle = std::acos( cos_angle );
//       return coord;
//     };
//     
//   Eigen::VectorXi row;
//   PointType x_axis_point = x_axis_projectory->points[0];
//   row.resize(l_*t_, 1);
//   for( int i = 0; i < l_*t_; ++i )
//     row[i] = 0;
//   const double double_pi = M_PI * 2.;
//   const double angle_step = double_pi / t_;
//   std::cout << x_axis_point << std::endl;
//   for( auto& point : projected_cloud->points )
//   {
//     PolarCoordinate coord = cal_polar(x_axis_point, point);
//     if( coord.angle < 0. )
//       coord.angle += double_pi;
//     
//     int32_t l_index = std::floor( std::sqrt( coord.length / r_ ) );
//     // 防止计算精度造成的问题，对index进行最大值的限制，防止越界
//     if( l_index > l_ - 1 ) 
//       l_index = l_ - 1; 
//     int32_t t_index = std::floor( coord.angle / angle_step );
//     if( t_index > t_ - 1 )
//       t_index = t_ - 1;
//     int32_t index = l_index*t_+t_index;
// //       std::cout << l_*t_ << "  " << index << "  " << l_index << "  " << t_index << "  " << coord.angle << std::endl;
//     row[index]++;
//   }
//   
//   return row;
// }

} // namespace back_end
} // namespace static_map
