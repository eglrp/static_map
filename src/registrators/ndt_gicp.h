#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifdef NDT_USE_GPU
#include "ndt_gpu/NormalDistributionsTransform.h"
#else
#include <pcl/registration/ndt.h>
#endif

#include "libicp/icpPointToPlane.h"

#include <vector>
#include <cmath>
#include "macro_defines.h"

namespace static_map
{

namespace front_end
{
  
struct SearchWindow
{
  double x, y, z;
  double x_resolution;
  double y_resolution;
  double z_resolution;
  double angle;
  double angle_resolution;
};
  
template <typename PointType>
class RegistrationWithNDTandGICP
{
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef pcl::PointCloud<PointType> PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;
  
public:
  
  RegistrationWithNDTandGICP( const SearchWindow& search_window, 
    bool using_voxel_filter, double voxel_resolution = 0.2)
    : input_cloud_( new PointCloudSource )
    , target_cloud_( new PointCloudTarget )
    , down_sampled_input_cloud_( new PointCloudSource )
    , down_sampled_target_cloud_( new PointCloudTarget )
    , voxel_resolution_( voxel_resolution )
    , using_voxel_filter_( using_voxel_filter )
    , ndt_search_window_( search_window )
  {
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt_.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt_.setStepSize( 0.1 );
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt_.setResolution(1.);
    //设置匹配迭代的最大次数
    ndt_.setMaximumIterations(35);
    
    gicp_.setRotationEpsilon(1e-3);
    gicp_.setMaximumIterations(35);
    

    approximate_voxel_filter_.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
    
    std::vector<double> x_search_candidates;
    if( ndt_search_window_.x_resolution < 0.1 )
      ndt_search_window_.x_resolution = 0.1;
    int32_t x_steps = ndt_search_window_.x / ndt_search_window_.x_resolution;
    if( x_steps < 0 )
      x_steps = -x_steps;
//     PCL_INFO("x steps :%d \n", x_steps);
    x_search_candidates.push_back( 0. );
    for( int32_t i = 1; i <= x_steps; ++i )
    {
      x_search_candidates.push_back( i * ndt_search_window_.x_resolution );
      x_search_candidates.push_back( -i * ndt_search_window_.x_resolution );
    }
    
    std::vector<double> y_search_candidates;
    if( ndt_search_window_.y_resolution < 0.1 )
      ndt_search_window_.y_resolution = 0.1;
    int32_t y_steps = ndt_search_window_.y / ndt_search_window_.y_resolution;
    if( y_steps < 0 )
      y_steps = -y_steps;
//     PCL_INFO("y steps :%d \n", y_steps);
    y_search_candidates.push_back( 0. );
    for( int32_t i = 1; i <= y_steps; ++i )
    {
      y_search_candidates.push_back( i * ndt_search_window_.y_resolution );
      y_search_candidates.push_back( -i * ndt_search_window_.y_resolution );
    }
    
    std::vector<double> z_search_candidates;
    if( ndt_search_window_.z_resolution < 0.1 )
      ndt_search_window_.z_resolution = 0.1;
    int32_t z_steps = ndt_search_window_.z / ndt_search_window_.z_resolution;
    if( z_steps < 0 )
      z_steps = -z_steps;
    z_search_candidates.push_back( 0. );
    for( int32_t i = 1; i <= z_steps; ++i )
    {
      z_search_candidates.push_back( i * ndt_search_window_.z_resolution );
      z_search_candidates.push_back( -i * ndt_search_window_.z_resolution );
    }
    
    std::vector<double> angle_search_candidates;
    if( ndt_search_window_.angle_resolution < 0.5 )
      ndt_search_window_.angle_resolution = 0.5;
    int32_t angle_steps = ndt_search_window_.angle / ndt_search_window_.angle_resolution;
    if( angle_steps < 0 )
      angle_steps = -angle_steps;
//     PCL_INFO("angle steps :%d \n", angle_steps);
    angle_search_candidates.push_back( 0. );
    for( int32_t i = 1; i <= angle_steps; ++i )
    {
      angle_search_candidates.push_back( i * ndt_search_window_.angle_resolution );
      angle_search_candidates.push_back( -i * ndt_search_window_.angle_resolution );
    }
    
    for( auto& x : x_search_candidates )
      for( auto& y : y_search_candidates )
        for( auto& z : z_search_candidates )
          for( auto& a : angle_search_candidates )
          {
            Eigen::AngleAxisf rotation( a/180*M_PI, Eigen::Vector3f::UnitZ());
            Eigen::Translation3f translation(x, y, z);
            Eigen::Matrix4f candidate = (translation * rotation).matrix();
            ndt_candidates_.push_back( candidate );
          }
    PCL_INFO("candidates count: %lu\n", ndt_candidates_.size());
  }
  
  ~RegistrationWithNDTandGICP() = default;
  
  typedef boost::shared_ptr< RegistrationWithNDTandGICP<PointType> > Ptr;
  typedef boost::shared_ptr< const RegistrationWithNDTandGICP<PointType> > ConstPtr;
  
  inline void
  setInputSource (const PointCloudSourceConstPtr &cloud)
  {
    if (cloud->points.empty ())
    {
      PCL_ERROR ("Invalid or empty point cloud dataset given!\n");
      return;
    }
    
    *input_cloud_ = *cloud;
  }
  
  inline void
  setInputTarget (const PointCloudTargetConstPtr &cloud)
  {
    if (cloud->points.empty ())
    {
      PCL_ERROR ("Invalid or empty point cloud dataset given!\n");
      return;
    }
    
    *target_cloud_ = *cloud;
  }
  
  bool align (const Eigen::Matrix4f& guess, Eigen::Matrix4f& result)
  {
    if( using_voxel_filter_ )
    {
      approximate_voxel_filter_.setInputCloud( input_cloud_ );
      approximate_voxel_filter_.filter(*down_sampled_input_cloud_);
      
      approximate_voxel_filter_.setInputCloud( target_cloud_ );
      approximate_voxel_filter_.filter(*down_sampled_target_cloud_);
    }
    else
    {
      *down_sampled_input_cloud_ = *input_cloud_;
      *down_sampled_target_cloud_ = *target_cloud_;
    }
    
    PointCloudSourcePtr output_cloud(new PointCloudSource);
    
    Eigen::Matrix4f ndt_guess = guess;
    double ndt_score = 0.9;
    if( use_ndt_ )
    {
      ndt_.setInputSource(down_sampled_input_cloud_);
      ndt_.setInputTarget(down_sampled_target_cloud_);
      
      #ifdef NDT_USE_GPU
      ndt_.align( guess );
      #else
      ndt_.align(*output_cloud, guess);
      #endif
      
      ndt_score = ndt_.getFitnessScore();
      
      #ifdef NDT_USE_GPU
      ndt_score /= 20;
      #endif

      ndt_guess = ndt_.getFinalTransformation();
    }
    
    double icp_score = 10.;
    Eigen::Matrix4f final_guess;
    if( ndt_score <= 1. )
    {  
      icp_score = ndt_score;
#ifdef USE_PCL_GICP
      
      gicp_.setInputSource( down_sampled_input_cloud_ );
      gicp_.setInputTarget( down_sampled_target_cloud_ );
      gicp_.align(*output_cloud, ndt_guess);
      
      icp_score = gicp_.getFitnessScore();
      final_guess = gicp_.getFinalTransformation();
      
      final_guess(2,3) = 0.;     
      
#else
      // libicp
      double* M = (double*)calloc(3*down_sampled_target_cloud_->size(),sizeof(double));
      double* T = (double*)calloc(3*down_sampled_input_cloud_->size(),sizeof(double));
      for( int i = 0; i < down_sampled_target_cloud_->size(); ++i )
      {
        M[i*3+0] = (*down_sampled_target_cloud_)[i].x;
        M[i*3+1] = (*down_sampled_target_cloud_)[i].y;
        M[i*3+2] = (*down_sampled_target_cloud_)[i].z;
      }
      for( int i = 0; i < down_sampled_input_cloud_->size(); ++i )
      {
        T[i*3+0] = (*down_sampled_input_cloud_)[i].x;
        T[i*3+1] = (*down_sampled_input_cloud_)[i].y;
        T[i*3+2] = (*down_sampled_input_cloud_)[i].z;
      }
      
//       ndt_guess
      Matrix R = Matrix::eye(3);
      for( int i = 0; i < 3; ++i )
        for( int j = 0; j < 3; ++j )
          R.val[i][j] = ndt_guess(i,j);
      
      Matrix t(3,1);
      t.val[0][0] = ndt_guess(0,3);
      t.val[1][0] = ndt_guess(1,3);
      t.val[2][0] = ndt_guess(2,3);
      
      IcpPointToPlane icp(M,down_sampled_target_cloud_->size(),3);
      icp_score = icp.fit(T,down_sampled_input_cloud_->size(),R,t,0.1);
      
      for( int i = 0; i < 3; ++i )
        for( int j = 0; j < 3; ++j )
          final_guess(i,j) = R.val[i][j];
      
      for( int i = 0; i < 3; ++i )
      {
        final_guess(i,3) = t.val[i][0];
        final_guess(3,i) = 0.;
      }
      final_guess(3,3) = 1.;
      
#endif

      final_score_ = std::exp( -icp_score );
      result = final_guess;
      return true;
    }
    else 
    {
      result = guess;
      final_score_ = std::exp( -icp_score );
      return false;
    }
    
    return true;
  }
  
  inline 
  double getFitnessScore() { return final_score_; }
  
  inline
  void enableNdt( bool use_ndt ){ use_ndt_ = use_ndt; }
  
private:
#ifdef NDT_USE_GPU
  gpu::GNormalDistributionsTransform ndt_;
#else 
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
#endif
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;
  
  
  PointCloudSourcePtr input_cloud_;
  PointCloudTargetPtr target_cloud_;
  
  PointCloudSourcePtr down_sampled_input_cloud_;
  PointCloudTargetPtr down_sampled_target_cloud_;
  
  pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter_;
  double voxel_resolution_;
  bool using_voxel_filter_;
  bool use_ndt_ = true;
  
  SearchWindow ndt_search_window_;
  std::vector<Eigen::Matrix4f> ndt_candidates_;
  
  double final_score_ = 0.;

};
}  // namespace front_end
}  // namespace static_map