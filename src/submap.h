#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>

#include "frame.h"

namespace static_map
{

struct SubmapId
{
  int32_t trajectory_index;
  int32_t submap_index;
};
  
template <typename PointType>
class Submap
{
public:
  
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
  
  Submap()
    : cloud( new PointCloudSource )
    , max_frame_size_( 5 )
    , full_( false )
    , enable_inner_multiview_icp_( false )
    {}
  ~Submap() = default;
  
  SubmapId id_;
  
  PointCloudSourcePtr cloud;
  Eigen::Matrix4f trans_to_global;
  Eigen::Matrix4f trans_to_next_submap;
  
  inline 
  void setMaxFrameSize( size_t size ){ max_frame_size_ = size; }
  
  void insertPointCloud( const PointCloudSourcePtr& pointcloud,
    Eigen::Matrix4f trans_to_submap)
  {
    if( full_ || pointcloud->empty() )
      return;
    
    auto frame = std::make_shared<Frame<PointType>>();
    frame->point_cloud_ = pointcloud;
    frame->trans_to_submap_ = trans_to_submap;
    frame->trans_to_global_ = trans_to_global * frame->trans_to_submap_;
    frame->id_.frame_index = frames_.size();
    frame->id_.submap_index = id_.submap_index;
    frame->id_.trajectory_index = id_.trajectory_index;
    frames_.push_back(frame);
    
    PointCloudSourcePtr output_cloud(new PointCloudSource);
    pcl::transformPointCloud( *pointcloud, *output_cloud, trans_to_submap );
    *cloud += *output_cloud;
    
    if( frames_.size() == max_frame_size_ )
      full_ = true;
    
    // TODO inner multiview icp
  }
  
  inline
  bool full() { return full_; }
  
  inline 
  void enableInnerMultiviewIcp( bool enable )
  {
    enable_inner_multiview_icp_ = enable;
  }
  
private:
  std::vector< std::shared_ptr<Frame<PointType>> > frames_;
  size_t max_frame_size_;
  
  bool full_;
  bool enable_inner_multiview_icp_;
};
  
}