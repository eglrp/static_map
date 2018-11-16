#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sstream>
#include "simple_time.h"
#include "map_builder.h"
#include "msg_conversion.h"

using static_map::MapBuilder;

MapBuilder::Ptr map_builder;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{ 
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL( *msg, pcl_pc2 );
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
    incoming_cloud( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2( pcl_pc2, *incoming_cloud );
  
//   static_map::back_end::M2dp<pcl::PointXYZ> m2dp;
//   
//   m2dp.setInputCloud( incoming_cloud );
  
//   m2dp.preProcess( incoming_cloud, output );
  
  map_builder->InsertPointcloudMsg( incoming_cloud );
}

void imu_callback( const sensor_msgs::Imu& imu_msg )
{
  sensors::ImuMsg::Ptr incomming_imu = boost::make_shared<sensors::ImuMsg>();
  *incomming_imu = sensors::ToLocalImu( imu_msg );
  
  map_builder->InsertImuMsg( incomming_imu );
}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "ndt_test_node");
  ros::NodeHandle n;

  ros::Subscriber sub_pointcloud = 
    n.subscribe("/velodyne_16/middle/velodyne_points", 1000, pointcloud_callback);
  ros::Subscriber sub_imu;
  
  ros::Rate loop_rate(1000);
  static_map::FrontEndSettings front_end_settings;
  static_map::BackEndSettings back_end_settings;
  front_end_settings.search_window = 
  {
    .x = 4., .y = 0.1001, .z = 0.,
    .x_resolution = 1.,
    .y_resolution = 0.1,
    .z_resolution = 0.1,
    .angle = 1.,
    .angle_resolution = 1.
  };
  front_end_settings.use_voxel_filter = true;
  front_end_settings.use_imu = true;
  front_end_settings.voxel_filter_resolution = 0.1;
  
  if( front_end_settings.use_imu )
    sub_imu = n.subscribe("/imu/data", 10000, imu_callback);
  
  map_builder = boost::make_shared<MapBuilder>();
  map_builder->Initialise( front_end_settings, back_end_settings );
  
  while( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  map_builder->RunFinalOptimazation();
  
  
  return 0;
}