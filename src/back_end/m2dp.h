#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <Eigen/SVD>  
#include <Eigen/Dense>

namespace static_map{
namespace back_end{
  
  
// refer to the paper
// "M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop
// Closure Detection"
template <typename PointType>
class M2dp
{
public:
  
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
  
  typedef Eigen::VectorXf Descriptor;
  
  M2dp( double r = 0.1, 
        double max_distance = 100.,
        int32_t t = 16, 
        int32_t p = 4, 
        int32_t q = 16 )
    : inner_cloud_( new PointCloudSource )
    , t_( t ) , p_( p ), q_( q ), r_( r )
    , max_distance_( max_distance )
    {}
  ~M2dp() = default;
  
  struct PolarCoordinate
  {
    double angle, length;
  };
  
  bool setInputCloud(const PointCloudSourcePtr& source)
  {
    if( source->empty() )
    {
      PCL_ERROR("source is empty.\n");
      return false;
    }
    
    // step1 pre processing
    preProcess( source );
    
    // step2 calculate A
    const double theta_step = M_PI / p_;
    const double phi_step = M_PI_2 / q_;
    for( int p = 0; p < p_; ++p )
      for( int q = 0; q < q_; ++q )
      {
        Eigen::VectorXi row = singleViewProcess( p * theta_step, q * phi_step );
        A_.block( p*q_+q, 0, 1, l_*t_ ) = row.transpose().cast<float>();
      }
    
    // SVD and get the final Descriptor
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A_, Eigen::ComputeThinU | Eigen::ComputeThinV );  
    Eigen::MatrixXf V = svd.matrixV(), U = svd.matrixU();  
    Eigen::VectorXf v1 = V.col(0), u1 = U.col(0);  // paper algorithm1.15
    
    descriptor_.resize(u1.rows()+v1.rows(), 1);
    descriptor_ << u1, v1;
//     std::cout << "descriptor_ :" << descriptor_.transpose() << std::endl;
    return true;
  }
  
  inline 
  Descriptor& getFinalDescriptor(){ return descriptor_; }
  
  
private:

  // part III.B in paper
  void preProcess( const PointCloudSourcePtr& source );
  // part III.C in paper
  Eigen::VectorXi singleViewProcess( double theta, double phi )
  {
      // normal
    Eigen::Vector3f m;
    m << std::cos(theta) * std::cos(phi), std::cos(theta) * std::sin(phi)
      , std::sin(theta);

    // refer to the matlab code in git (according to the paper)
    Eigen::Vector3f projected_x_axis = Eigen::Vector3f(1.,0.,0.) - 
      ( Eigen::Vector3f(1.,0.,0.).transpose() * m ).norm() * m;
    Eigen::Vector3f projected_y_axis = m.cross( projected_x_axis );
    
    const double double_pi = M_PI * 2.;
    auto calulate_polar =
      [ &double_pi ](const Eigen::Vector2f& p)
      {
        PolarCoordinate coord;
        coord.length = p.norm();
        coord.angle = std::atan2(p[1],p[0]);
        if( coord.angle < 0. )
          coord.angle += double_pi;
        
        return coord;
      };
    
    Eigen::VectorXi row;
    row.resize(l_*t_, 1);
    for( int i = 0; i < l_*t_; ++i )
      row[i] = 0;
    const double angle_step = double_pi / t_;
    for( auto& point : inner_cloud_->points )
    {
      Eigen::Vector3f p( point.x, point.y, point.z );
      Eigen::Vector2f projected_point( 
        ( p.transpose() * projected_x_axis ).norm(),
        ( p.transpose() * projected_y_axis ).norm() );
      
      PolarCoordinate coord = calulate_polar( projected_point );
      int32_t l_index = std::floor( std::sqrt( coord.length / r_ ) );
      // 防止计算精度造成的问题，对index进行最大值的限制，防止越界
      if( l_index > l_ - 1 ) 
        l_index = l_ - 1; 
      int32_t t_index = std::floor( coord.angle / angle_step );
      if( t_index > t_ - 1 )
        t_index = t_ - 1;
      int32_t index = l_index*t_+t_index;
//         std::cout << index << "  " << l_index << "  " << t_index << "  " << coord.angle << std::endl;
      row[index]++;
    }
    
    return row;
  }
  
  inline 
  double getLength( const PointType& a )
  { return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z); }
  
  PointCloudSourcePtr inner_cloud_;
  
  Eigen::Vector3f mean_;
  
  // parameters in paper
  int32_t l_, t_;
  int32_t p_, q_;
  double r_; 
  double max_distance_;
  
  Eigen::MatrixXf A_;
  Descriptor descriptor_;
};

// return the match score (-1,1)
template <typename PointType>
double matchTwoM2dpDescriptors( const typename M2dp<PointType>::Descriptor& P,
                                const typename M2dp<PointType>::Descriptor& Q)
{
  if( P.rows() != P.rows() )
  {
    PCL_ERROR("The Descriptors do not match.\n");
    return -1.;
  }
  
  // refer to paper
  // "Spin-Images: A Representation for 3-D Surface Matching"
  // section 2.4 page.28
  auto N = P.rows();
  double score = ( N*P.dot(Q)-P.sum()*Q.sum() )
    / std::sqrt( (N*P.dot(P)-std::pow(P.sum(),2)) * (N*Q.dot(Q)-std::pow(Q.sum(),2)));
  
  return score;
}
  
} // namespace back_end
} // namespace static_map

#include "m2dp_impl.hpp"