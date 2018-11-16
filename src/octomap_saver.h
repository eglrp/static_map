#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include "macro_defines.h"

namespace static_map
{
  
namespace back_end
{

template <typename PointType>
class OctomapSaver
{
public:
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
  
  OctomapSaver( double resolution = 0.05)
    : tree_( resolution < 0.01 ? 0.01 : resolution )    /* 限制最小分辨率为 1cm */
  {
    tree_.setProbHit( 0.55 );
    tree_.setProbMiss( 0.48 );
  }
  ~OctomapSaver() = default;
  
  void insertPointCloud( PointCloudSourcePtr& cloud, Eigen::Vector3d origin )
  {
    if( cloud->empty() )
      return;
    
    octomap::Pointcloud octo_cloud;
    for( auto& point : cloud->points )
      octo_cloud.push_back(point.x, point.y, point.z);
    
    tree_.insertPointCloud( octo_cloud, 
      octomap::point3d( origin(0), origin(1), origin(2) ) );
    
    tree_.updateInnerOccupancy();
  }
  
  void writeToFile( const std::string& filename )
  {
    tree_.write( filename );
  }
  
  void saveOccupanyPCD( const std::string& filename, double threshold = 0.6 )
  {
    double max_occu = 0.;
    PointCloudSourcePtr pointcloud( new PointCloudSource );
    
    for(octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
       end=tree_.end_leafs(); it!= end; ++it)
    {
      if( (*it).getOccupancy() > max_occu )
      {
        max_occu = (*it).getOccupancy();
      }
      
      if( (*it).getOccupancy() > threshold )
      {
        PointType point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();
        pointcloud->push_back( point );
      }
    }
    std::cout << "max_occu = " << max_occu << std::endl;
    
    if( pointcloud->empty() )
    {
      PRINT_ERROR("The final point cloud is empty");
      return;
    }
    pcl::io::savePCDFileASCII( filename, *pointcloud );
  }
  
private:
  octomap::OcTree tree_;
};
  
}

  
}  // namespace static_map