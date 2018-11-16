#pragma once

#include "clusterers/image_based_clusterer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "registrators/ndt_gicp.h"
#include "msg_conversion.h"
#include "cloud_corrector.h"
#include <mutex>
#include <thread>
#include <memory>

// gtsam
// used for imu preintergration
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include "octomap_saver.h"
#include "pose_graph.h"
#include "submap.h"

#include "common/thread_pool.h"

namespace static_map
{

struct FrontEndSettings
{
  front_end::SearchWindow search_window;
  bool use_voxel_filter;
  double voxel_filter_resolution;
  
  bool use_imu;
};

struct BackEndSettings
{
  int32_t reserved[20];
};
  
class MapBuilder
{
public:
  MapBuilder();
  ~MapBuilder();
  
  typedef boost::shared_ptr< MapBuilder > Ptr;
  typedef boost::shared_ptr< const MapBuilder > ConstPtr;
  
  inline
  void InsertPointcloudMsg( 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud )
  {
    // 如果已经设定关闭所有线程，这里就不再接受新的数据写入
    if( end_all_thread_ )
      return;
    
    mutex_for_point_cloud_.lock();
    point_clouds_.push_back( point_cloud );
    mutex_for_point_cloud_.unlock();
  }
  
  inline 
  void InsertImuMsg( const sensors::ImuMsg::Ptr& imu_msg )
  {
    if( end_all_thread_ )
      return;
    
    mutex_for_imu_.lock();
    imus_.push_back( imu_msg );
    mutex_for_imu_.unlock();
  }
  
  int32_t Initialise( FrontEndSettings& f_setting, BackEndSettings& b_setting );
  
  void PointcloudProcessing();
  
  void ImuProcessing();
  
  void SubmapProcessing();
  
  void RunFinalOptimazation();
  
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr
  GetFinalPointcloud(){ return final_cloud_ ;}
  
protected:
  
  void DownSamplePointcloud( pcl::PointCloud<pcl::PointXYZ>::Ptr source
    , pcl::PointCloud<pcl::PointXYZ>::Ptr output
    , double xy_range = 80., double z_negetive = -1.5, double z_positive = 3.);
  
  back_end::Pose3d ToPose( Eigen::Matrix4d trans );
  Eigen::Matrix4d ToTransform( const back_end::Pose3d& pose );
  
  void imu_estimate_on_time( SimpleTime stamp );
  
  double GetMatchScore( pcl::PointCloud< pcl::PointXYZ >::Ptr source, 
                             pcl::PointCloud< pcl::PointXYZ >::Ptr target, 
                             const Eigen::Matrix4f& transfrom );
  
  void DepthClustering( pcl::PointCloud< pcl::PointXYZ >::Ptr source, 
    pcl::PointCloud< pcl::PointXYZL >::Ptr output );
  
private:
  typedef Eigen::Matrix<double, 6, 6> InformationMatrix;
  
  std::mutex mutex_for_point_cloud_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds_;
  
  bool use_imu_;
  std::mutex mutex_for_imu_;
  std::vector<sensors::ImuMsg::Ptr> imus_;
  
  bool end_all_thread_ = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_;
  
  // frond end
  front_end::RegistrationWithNDTandGICP<pcl::PointXYZ>::Ptr registration_;
  std::shared_ptr<std::thread> point_cloud_process_thread_;
  bool point_cloud_thread_running_;
  std::shared_ptr<std::thread> imu_process_thread_;
  bool got_first_point_cloud_ = false;
  
  front_end::CloudCorrector<pcl::PointXYZ> cloud_corrector_;
  
  // imu preintergration
  gtsam::imuBias::ConstantBias bias_;
  gtsam::CombinedImuFactor::CombinedPreintegratedMeasurements 
    imu_current_estimate_;
    
  // back end
  std::mutex point_cloud_submap_mutex_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds_for_submaps_;
  std::mutex transform_submap_mutex_;
  std::vector<Eigen::Matrix4f> transfroms_for_submaps_;
  std::vector<InformationMatrix> information_matrices_;
  front_end::RegistrationWithNDTandGICP<pcl::PointXYZ>::Ptr submap_marcher_;
  bool submap_thread_running_;
  std::vector<Submap<pcl::PointXYZ>> submaps_;
  std::shared_ptr<std::thread> submap_thread_;
  
  back_end::OctomapSaver<pcl::PointXYZ> octomap_saver_;
};
  
}