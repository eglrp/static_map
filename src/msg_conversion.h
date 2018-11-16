#pragma once

#include "sensors.h"
#include <sensor_msgs/Imu.h>

namespace sensors
{

typedef uint64_t PclTimeStamp;
SimpleTime ToLocalTime( const ros::Time& time );
SimpleTime ToLocalTime( const PclTimeStamp& time );

Header ToLocalHeader( const std_msgs::Header& header );
ImuMsg ToLocalImu( const sensor_msgs::Imu& msg );
  
}