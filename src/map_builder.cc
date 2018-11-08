#include "map_builder.h"
#include "macro_defines.h"
#include "multiview_icp_ceres.h"

#include <chrono>
#include <boost/make_shared.hpp>
#include <pcl/filters/conditional_removal.h>

#include "projections/projection_params.h"
#include "ground_removal/depth_ground_remover.h"
#include "clusterers/image_based_clusterer.h"
#include "image_labelers/linear_image_labeler.h"
#include "utils/cloud_saver.h"


namespace static_map
{

MapBuilder::MapBuilder()
  : use_imu_( true )
  , registration_( nullptr )
  , point_cloud_process_thread_( nullptr )
  , imu_process_thread_( nullptr )
  , bias_( gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0))
  , imu_current_estimate_(bias_,
      gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(),
      gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(), gtsam::Matrix::Zero(6,6))
  {}
  
int32_t MapBuilder::Initialise( FrontEndSettings& f_setting, 
                                BackEndSettings& b_setting )
{
  if( registration_ == nullptr )
    registration_ = boost::make_shared<
      front_end::RegistrationWithNDTandGICP<pcl::PointXYZ>>( 
        f_setting.search_window, f_setting.use_voxel_filter, 
        f_setting.voxel_filter_resolution );
  else
  {
    PRINT_ERROR("Has been inited already.");
    return -1;
  }
  
  submap_marcher_ = boost::make_shared<
      front_end::RegistrationWithNDTandGICP<pcl::PointXYZ>>( 
        f_setting.search_window, false );
  
  final_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  point_cloud_process_thread_ = std::make_shared<std::thread>(
    std::bind(&MapBuilder::PointcloudProcessing, this) );
  
  use_imu_ = f_setting.use_imu;
//   if( f_setting.use_imu )
//     imu_process_thread_ = std::make_shared<std::thread>(
//       std::bind(&MapBuilder::ImuProcessing, this) );
    
  submap_thread_ = std::make_shared<std::thread>(
    std::bind(&MapBuilder::SubmapProcessing, this) );
  
  // b_setting
  (void)b_setting;
  
  return 0;
}

MapBuilder::~MapBuilder()
{
}

void MapBuilder::PointcloudProcessing()
{
  point_cloud_thread_running_ = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  
  Eigen::AngleAxisf init_rotation( 0 , Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0, 0, 0);
  Eigen::Matrix4f last_guess = (init_translation * init_rotation).matrix();
  Eigen::Matrix4f final_transform = (init_translation * init_rotation).matrix();
  
  gtsam::Rot3 last_rotation;
  bool last_match_is_good = true;
  while( true )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ> );
    
    mutex_for_point_cloud_.lock();
    if( !point_clouds_.empty() )
    {
      if( !got_first_point_cloud_ )
      {
        *target_cloud = *point_clouds_.front();
        point_clouds_.erase( point_clouds_.begin() );
        got_first_point_cloud_ = true;
        mutex_for_point_cloud_.unlock();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud(
          new pcl::PointCloud<pcl::PointXYZ> );
        *first_cloud = *target_cloud;
        point_cloud_submap_mutex_.lock();
        point_clouds_for_submaps_.push_back( first_cloud );
        point_cloud_submap_mutex_.unlock();
        
        continue;
      }
      
      *input_cloud = *point_clouds_.front();
      point_clouds_.erase( point_clouds_.begin() );
      mutex_for_point_cloud_.unlock();
    }
    else
    {
      mutex_for_point_cloud_.unlock();
      if( end_all_thread_ )
        break;      
      
      usleep(10000);
      continue;
    }
    
    // down sample
    DownSamplePointcloud( input_cloud, input_cloud_filtered, 80., -3., 3. );
    DownSamplePointcloud( target_cloud, target_cloud_filtered, 80., -3., 3.);
    registration_->setInputSource( input_cloud_filtered );
    registration_->setInputTarget( target_cloud_filtered );
    
    Eigen::Matrix4f align_result;
    Eigen::Matrix4f guess = last_guess;
    
    double delta_yaw = 0.;
    if( use_imu_ )
    { 
      last_rotation = imu_current_estimate_.deltaRij();
      imu_estimate_on_time( sensors::ToLocalTime( input_cloud->header.stamp ) );
      
      delta_yaw = imu_current_estimate_.deltaRij().yaw() - last_rotation.yaw();
      Eigen::AngleAxisf delta_rotation( delta_yaw , Eigen::Vector3f::UnitZ() );
      guess.block(0, 0, 3, 3) = delta_rotation.matrix();
    }
    
    if( registration_->align( guess, align_result ) )
    {
      if( last_match_is_good )
        last_guess = align_result;
      else
        last_guess = (init_translation * init_rotation).matrix();
      
      double match_score = registration_->getFitnessScore();
      
      Eigen::Quaternionf q_quess = Eigen::Quaternionf( Eigen::Matrix3f( guess.block(0,0,3,3) ) );
      Eigen::Quaternionf q = Eigen::Quaternionf( Eigen::Matrix3f( align_result.block(0,0,3,3) ) );
      
      double q_match_score = q.dot(q_quess) * 0.5;
      const double move_threshold = 0.15;
      if( align_result.block(0,3,3,1).norm() > move_threshold ) 
      {
        point_cloud_submap_mutex_.lock();
        point_clouds_for_submaps_.push_back( input_cloud );
        point_cloud_submap_mutex_.unlock();

        final_transform *= align_result;
        pcl::transformPointCloud(*input_cloud, *output_cloud, final_transform);
        *final_cloud_ += *output_cloud;

        InformationMatrix information = InformationMatrix::Identity();
        const double t = 180. / M_PI;
        for( int i = 0; i < 6; ++i )
        {
          if( i < 3 )
            information(i,i) = match_score;
          else
            information(i,i) = q_match_score /** ( 1. - fabs(delta_yaw*t) * 0.1 )*/;
        }
        
        transform_submap_mutex_.lock();
        transfroms_for_submaps_.push_back(align_result);
        information_matrices_.push_back(information);
        transform_submap_mutex_.unlock();
        
        target_cloud->clear();
        *target_cloud = *input_cloud;
      }
    }
    else
    {
      last_match_is_good = false;
    }
  }
  
  point_cloud_thread_running_ = false;
  PRINT_INFO("point cloud thread exit.");
}

void MapBuilder::ImuProcessing()
{
//   sensors::ImuMsg current_imu_data;
//   while( true )
//   {
//     mutex_for_imu_.lock();
//     if( !imus_.empty() )
//     {
//       current_imu_data = *imus_.front();
//       imus_.erase( imus_.begin() );
//     }
//     else
//     {
//       mutex_for_imu_.unlock();
//       
//       if( end_all_thread_ )
//         break;      
//       
//       std::this_thread::sleep_for( std::chrono::microseconds(5) );
//       continue;
//     }
//     mutex_for_imu_.unlock();
//   }
  
  PRINT_INFO("imu thread exit.");
}

void MapBuilder::imu_estimate_on_time( SimpleTime stamp )
{
  std::vector<sensors::ImuMsg::Ptr> used_imus;
  {
    mutex_for_imu_.lock();
    
    if( imus_.empty() || imus_.front()->header.stamp >= stamp )
    {
      mutex_for_imu_.unlock();
      return;
    }
    
    if( imus_.back()->header.stamp <= stamp )
      std::swap( used_imus, imus_ );
    else
      while( imus_.front()->header.stamp <= stamp )
      {
        used_imus.push_back( imus_.front() );
        imus_.erase( imus_.begin() );
      }
    
    mutex_for_imu_.unlock();
  }
  
  if( used_imus.empty() )
    return;
  
  for( auto& imu : used_imus )
  {
    gtsam::Vector3 acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    gtsam::Vector3 angle_velocity(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    imu_current_estimate_.integrateMeasurement( acc, angle_velocity, 0.01 );
  }
//   std::cout << imu_current_estimate_.deltaPij()[0] << ", " << imu_current_estimate_.deltaPij()[1] << ", " << imu_current_estimate_.deltaPij()[2] << std::endl;
}

back_end::Pose3d MapBuilder::ToPose( Eigen::Matrix4d trans )
{
  back_end::Pose3d pose;
  pose.p = trans.block(0,3,3,1);
  ::Eigen::Matrix3d R = trans.block(0,0,3,3);
  pose.q = R;
  pose.q.normalize();
  
  return pose;
}

Eigen::Matrix4d MapBuilder::ToTransform( const back_end::Pose3d& pose )
{
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R = pose.q.toRotationMatrix();
  trans.block(0,0,3,3) = R;
  trans.block(0,3,3,1) = pose.p;
  
  return trans;
}

void MapBuilder::SubmapProcessing()
{
  submap_thread_running_ = true;
  
  const int32_t submap_cloud_count = 5;
  const int32_t transform_count = submap_cloud_count - 1;
  std::vector<std::pair<int, int>> contraint_pairs;
  int source = 0, target = 1;
  while( true )
  {
    contraint_pairs.push_back( std::make_pair( source, target ));
    target++;
    if(target > submap_cloud_count-1)
    {
      source++;
      target = source+1;
      if( target > submap_cloud_count-1)
        break;
    }
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_point_clouds;
  std::vector<Eigen::Matrix4f> local_transfroms;
  std::vector<InformationMatrix> local_informations;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  
  back_end::Pose3d init_pose;
  init_pose.p << 0., 0., 0.;
  init_pose.q = Eigen::Quaterniond::Identity();
  bool is_first_submap = true;
  while(true)
  { 
    point_cloud_submap_mutex_.lock();
    if( point_clouds_for_submaps_.size() < submap_cloud_count )
    {
      point_cloud_submap_mutex_.unlock();
      if( !point_cloud_thread_running_ )
        break;
      
      SimpleTime::from_sec(0.5).sleep();
      continue;
    }
    
    transform_submap_mutex_.lock();
    if( transfroms_for_submaps_.size() < transform_count )
    {
      transform_submap_mutex_.unlock();
      SimpleTime::from_sec(0.5).sleep();
      continue;
    }
    
    if( is_first_submap )
    {
      is_first_submap = false;
      local_point_clouds.push_back( point_clouds_for_submaps_.front() );
      point_clouds_for_submaps_.erase( point_clouds_for_submaps_.begin() );
    }
    
    for( int i = 0; i < transform_count; ++i )
    {
      local_point_clouds.push_back( point_clouds_for_submaps_.front() );
      point_clouds_for_submaps_.erase( point_clouds_for_submaps_.begin() );
      local_transfroms.push_back( transfroms_for_submaps_.front() );
      transfroms_for_submaps_.erase( transfroms_for_submaps_.begin() );
      
      local_informations.push_back( information_matrices_.front() );
      information_matrices_.erase( information_matrices_.begin() );
    }
    transform_submap_mutex_.unlock();
    point_cloud_submap_mutex_.unlock();
    
    if( local_point_clouds.size() != submap_cloud_count
      || local_point_clouds.size() != local_transfroms.size() + 1
      || local_informations.size() != transform_count )
    {
      PRINT_ERROR("Wrong size!");
      break;
    }
    
    // 建立小的回环
    // 先确定所有的节点
    static_map::back_end::MapOfPoses poses;
    static_map::back_end::VectorOfConstraints constraints;
    
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    poses[0] = init_pose;  
    
    int i = 1;
    for( auto& trans : local_transfroms )
    {
      guess *= trans;
      poses[i] = ToPose( guess.cast<double>() );
      i++;
    }
    
    // 添加新的 constraints
    for( auto& cp : contraint_pairs )
    {
      back_end::Constraint3d new_constraint;
      new_constraint.id_begin = cp.first;
      new_constraint.id_end = cp.second;
      new_constraint.information = Eigen::Matrix< double, 6, 6 >::Identity();
      Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
      if( cp.second == cp.first + 1 )
      {
        trans = local_transfroms[cp.first];
        new_constraint.information = local_informations[cp.first];
      }
      else
      {
        // 先计算不相邻两个点云的预测变换矩阵
        for( int i = cp.first; i < cp.second; ++i )
          trans *= local_transfroms[i];
        
        // 过滤掉一些角度或者位移过大的匹配，因为有可能会造成失败
        const double move_threshold = 1.5;
        
        if( trans.block(0,3,3,1).norm() > move_threshold ) 
          continue;
        
        DownSamplePointcloud( local_point_clouds.at(cp.first), target_cloud_filtered );
        DownSamplePointcloud( local_point_clouds.at(cp.second), input_cloud_filtered );
        
        submap_marcher_->setInputSource( input_cloud_filtered );
        submap_marcher_->setInputTarget( target_cloud_filtered );
        Eigen::Matrix4f result;
        submap_marcher_->align( trans, result );
        trans = result;

        double match_score = submap_marcher_->getFitnessScore();
        PRINT_DEBUG_FMT("contraint_pairs score: %lf", match_score );
        if( match_score < 0.85 )
          continue;
        
        InformationMatrix& information = new_constraint.information;
        for( int i = 0; i < 6; ++i )
        {
          if( i < 3 )
            information(i,i) = match_score;
          else
            information(i,i) = match_score/* * 0.5*/;
        }
        
      }
      new_constraint.t_be = ToPose( trans.cast<double>() );
      constraints.push_back( new_constraint );
    }

    std::cout << "**** \n" << poses[4].q.x() << " " << poses[4].q.y()
     << " " << poses[4].q.z() << " " << poses[4].q.w()<< std::endl;
//     ceres::Problem problem;
//     back_end::BuildOptimizationProblem(constraints, &poses, &problem);
//     back_end::SolveOptimizationProblem(&problem);
    std::cout << "==== \n" << poses[4].q.x() << " " << poses[4].q.y()
     << " " << poses[4].q.z() << " " << poses[4].q.w()<< std::endl;
    
    submaps_.emplace_back();
    auto& submap = submaps_.back();
    submap.index = submaps_.size() - 1;
    submap.cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for( int i = 0; i < poses.size(); ++i )
    {
      Eigen::Matrix4d transform = ToTransform( poses.at(i) );
      pcl::transformPointCloud( *local_point_clouds.at(i), *output_cloud, transform );
      *submap.cloud += *output_cloud;
    }
    
    submap.trans_to_next_submap =  
      ToTransform( poses[submap_cloud_count-1] ).cast<float>();
    if( submap.index == 0 )
      submap.trans_to_global = Eigen::Matrix4f::Identity();
    else
    {
      submap.trans_to_global = submaps_.at(submap.index-1).trans_to_global;
      submap.trans_to_global *= submaps_.at(submap.index-1).trans_to_next_submap;
      
/*      
      submap.trans_to_global = submaps_.at(submap.index-1).trans_to_global *
        submaps_.at(submap.index-1).trans_to_next_submap;*/
    }
        
    std::string filename = "submap_" + std::to_string( submap.index ) + ".pcd";
    pcl::io::savePCDFileASCII(filename, *submap.cloud);
    
    if( submap.index == 15 )
    {
      int test_cloud = 0;
      for( auto& cloud : local_point_clouds )
      {
        std::string file = "test_cloud_" + std::to_string( test_cloud ) + ".pcd";
        pcl::io::savePCDFileASCII(file, *cloud);
        test_cloud++;
      }
    }
    
    // 数据清理
    local_point_clouds.erase( local_point_clouds.begin(), 
      local_point_clouds.begin() + submap_cloud_count - 1);
    if( local_point_clouds.size() != 1 )
      PRINT_ERROR("Wrong size!");
    
    local_transfroms.clear();
    local_informations.clear();
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  
  for( auto& submap : submaps_ )
  { 
//     // 对新建的 submap 重新做一次匹配，得到更好的结果后接入到最终的地图里面去
//     if( submap.index > 0 )
//     {
//       auto& last_submap = submaps_.at(submap.index-1);
//   
//       DownSamplePointcloud( last_submap.cloud, target_cloud_filtered );
//       DownSamplePointcloud( submap.cloud, input_cloud_filtered );
//       
//       submap_marcher_->setInputSource( input_cloud_filtered );
//       submap_marcher_->setInputTarget( target_cloud_filtered );
//       Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
// 
//       submap_marcher_->align( last_submap.trans_to_next_submap, guess );
//       
//       double match_score = submap_marcher_->getFitnessScore();
//       
//       std::cout << "submap match score: " << match_score << std::endl;
//       if( match_score > 0.9 )
//       {
//         last_submap.trans_to_next_submap = guess;
//       }
//       else
//       { // do nothing, remains the former transform 
//         std::cout << "remains the former transform" << std::endl;
//       }
//       
//       // 不管怎样，global都是要更新的，以为前面的 transfrom 可能更新过
//       submap.trans_to_global = last_submap.trans_to_global 
//         * last_submap.trans_to_next_submap;
//     }
    pcl::transformPointCloud( *submap.cloud, *output_cloud, submap.trans_to_global );
    *final_cloud += *output_cloud;
  }
  
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  approximate_voxel_filter.setInputCloud( final_cloud );
  approximate_voxel_filter.filter(*filtered_final_cloud);
  pcl::io::savePCDFileASCII("final_optimized.pcd", *filtered_final_cloud);
  
  PRINT_INFO("submap thread exit.");
  submap_thread_running_ = false;
}

void MapBuilder::RunFinalOptimazation()
{ 
  PRINT_INFO("Running Final Optimazation...");
  end_all_thread_ = true; 
  size_t remaining_pointcloud_count = point_clouds_.size();
  int delay = 0;
  SimpleTime delay_time = SimpleTime::from_sec( 0.1 );
  if( point_cloud_process_thread_ )
  {
    do
    {
      delay_time.sleep();
      delay++;
      if( delay >= 20 )
      {
        std::cout << "Optimazing : " 
          << ( 1. - (double)point_clouds_.size()  / remaining_pointcloud_count ) 
            * 100. << "%" << std::endl;
        delay = 0;
      }
      
    }while( point_cloud_thread_running_ );
    
    point_cloud_process_thread_->join();
  }
  
  if( imu_process_thread_ )
  {
    do
    {
      std::this_thread::sleep_for( std::chrono::microseconds(100) );
    }while( !imu_process_thread_->joinable() );
    
    imu_process_thread_->join();
  }
  
  if( submap_thread_ )
  {
    do
    {
      delay_time.sleep();
    }while( submap_thread_running_ );
    submap_thread_->join();
  }
}

static int test = 0;
void MapBuilder::DownSamplePointcloud( pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr output, 
    double xy_range, double z_negetive, double z_positive  )
{
  (void)xy_range; (void)z_negetive; (void)z_positive;
  
  std::vector<int> mapping;  // no use, just for the function
  pcl::removeNaNFromPointCloud(*source, *source, mapping);
  
  // 这个 xyz 方向上的限制实际上是一个 trick，本来是用来剔除地面以及过远的散点对于匹配的影响
  // 现在弃用这个 trick，因为以后我们的雷达可能并非水平放置，这个trick没法用
  // 采用更合理的 统计滤波 的方式来解决散点的问题
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());   
//   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//     pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -xy_range)));   
//   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//     pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, xy_range)));
//   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//     pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -xy_range)));   
//   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//     pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, xy_range)));
  
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;//创建条件滤波器
  condrem.setCondition (range_cond); //并用条件定义对象初始化            
  condrem.setInputCloud (source);     //输入点云
  condrem.setKeepOrganized(true);    //设置保持点云的结构
  // 执行滤波
  condrem.filter(*output); 
  
  // 统计滤波
//   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//   sor.setInputCloud ( source );
//   sor.setMeanK ( 30 );                               
//   sor.setStddevMulThresh ( 1. );                  
//   sor.filter (*output); 
//   
//   if( test == 0 )
//   {
//     pcl::io::savePCDFile( "tes1.pcd" , *output );
//     test = 1;
//   }
  
  
//   // 最小二乘重构
//   // 创建 KD-Tree
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//   // 定义最小二乘实现的对象mls
//   pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//  
//   mls.setComputeNormals (true);  //设置在最小二乘计算中需要进行法线估计
// 
//   // Set parameters
//   mls.setInputCloud (filtered2);
//   mls.setPolynomialFit (true);
//   mls.setSearchMethod (tree);
//   mls.setSearchRadius (0.1);
// 
//   // Reconstruct
//   mls.process ( *output );
}

double MapBuilder::GetMatchScore( 
  pcl::PointCloud<pcl::PointXYZ>::Ptr source,
  pcl::PointCloud<pcl::PointXYZ>::Ptr target, const Eigen::Matrix4f& transfrom )
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud( new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud( *source, *output_cloud, transfrom );
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud( target );
  double search_radius = 0.1;
  
  std::vector<int> point_index_radius_search;
  std::vector<float> point_distance;
  int32_t match_count = 0;
  for( int i = 0; i < output_cloud->size(); ++i )
  {
    auto& point = (*output_cloud)[i];
    if( kdtree.radiusSearch ( point, search_radius , point_index_radius_search, point_distance) > 0 )
      match_count++;
  }
  
  return (double)match_count / (double)output_cloud->size();
}

using namespace depth_clustering;
void MapBuilder::DepthClustering( pcl::PointCloud< pcl::PointXYZ >::Ptr source, 
    pcl::PointCloud< pcl::PointXYZL >::Ptr output )
{
  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  int smooth_window_size = 9;
  depth_clustering::Radians ground_remove_angle = 7_deg;

  auto proj_params_ptr = depth_clustering::ProjectionParams::VLP_16();

  auto depth_ground_remover = depth_clustering::DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);
  auto cloud_saver = depth_clustering::CloudSaver("clustering");
  
  Radians angle_tollerance = 10_deg;
  
  ImageBasedClusterer<LinearImageLabeler<>> clusterer(
      angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  depth_ground_remover.AddClient(&clusterer);
  
  if( visualizer_ )
    clusterer.AddClient(visualizer_->object_clouds_client());
  
  std::cout << "source size: " << source->size() << std::endl;
  
  auto cloud = Cloud::FromPcl( *source );
  cloud->InitProjection(*proj_params_ptr);
  if( visualizer_ )
   visualizer_->OnNewObjectReceived(*cloud, 0);
  depth_ground_remover.OnNewObjectReceived(*cloud, 0);
  
  output = cloud->ToPcl();
  
  std::cout << "output size: " << output->size() << std::endl;
  cloud_saver.OnNewObjectReceived(*cloud, 0);
  
  usleep( 10000000 );
  
}
  
}