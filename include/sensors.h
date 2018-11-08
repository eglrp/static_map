#pragma once

#include "simple_time.h"
#include <boost/shared_ptr.hpp>

namespace sensors
{
  
typedef struct {
  double x;
  double y;
  double z;
  double w;
} Quaternion;

struct Vector3 {
public:
  Vector3() { x = y = z = 0.; }

  Vector3(double x_, double y_, double z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }

  double x, y, z;

  inline double distance_to_vector3(const Vector3 &vector)
  {
    return sqrt(pow(vector.x - x, 2.) + pow(vector.y - y, 2.)
                + pow(vector.z - z, 2.));
  }

  inline double distance_to_point3d(const Vector3 &vector)
  {
    return distance_to_vector3(vector);
  }

  void set_xy(double d_x, double d_y)
  {
    x = d_x;
    y = d_y;
  }

  void set_xyz(double d_x, double d_y, double d_z)
  {
    x = d_x;
    y = d_y;
    z = d_z;
  }

  Vector3 mid_point_with(const Vector3 &vector)
  {
    Vector3 point;
    point.set_xyz((vector.x + x) * 0.5, (vector.y + y) * 0.5,
                  (vector.z + z) * 0.5);
    return point;
  }

  Vector3 operator+(const Vector3 &b)
  {
    return Vector3(x + b.x, y + b.y, z + b.z);
  }

  Vector3 operator-(const Vector3 &b)
  {
    return Vector3(x - b.x, y - b.y, z - b.z);
  }

  std::string DebugString() const
  {
    std::ostringstream out;
    out << x << "," << y << "," << z;
    return out.str();
  }
};
typedef Vector3 Point;
typedef Vector3 Point3d;
typedef Vector3 translation;

struct Header 
{
public:
  Header() = default;
  ~Header() = default;
  
  Header &operator=(const Header &header)
  {
    seq = header.seq;
    stamp = header.stamp;
    frame_id = header.frame_id;

    return *this;
  }

  uint32_t seq;
  SimpleTime stamp;
  std::string frame_id;
};


struct ImuMsg 
{
public:
  ImuMsg() = default;
  ~ImuMsg() = default;
  
//   ImuMsg &operator=(const ImuMsg &msg)
//   {
//     header = msg.header;
//     
//     angular_velocity.x = msg.angular_velocity.x;
//     angular_velocity.y = msg.angular_velocity.y;
//     angular_velocity.z = msg.angular_velocity.z;
//     
//     linear_acceleration.x = msg.linear_acceleration.x;
//     linear_acceleration.y = msg.linear_acceleration.y;
//     linear_acceleration.z = msg.linear_acceleration.z;
//     
//     orientation.x = msg.orientation.x;
//     orientation.y = msg.orientation.y;
//     orientation.z = msg.orientation.z;
//     orientation.w = msg.orientation.w;
//     
//     for( int i = 0; i < 9; ++i )
//     {
//       angular_velocity_covariance[i] = msg.angular_velocity_covariance[i];
//       linear_acceleration_covariance[i] = msg.linear_acceleration_covariance[i];
//       orientation_covariance[i] = msg.orientation_covariance[i];
//     }
//   
//     return *this;
//   }
  
  typedef double Convariance[9];
  
  Header header;

  // gesture
  Quaternion orientation;
  Convariance orientation_covariance;

  // angular_velocity
  // rad/s
  Vector3 angular_velocity;
  Convariance angular_velocity_covariance;

  // linear_acceleration
  // m/s^2
  Vector3 linear_acceleration;
  Convariance linear_acceleration_covariance;

  double roll;
  double pitch;
  double yaw;
  
  typedef boost::shared_ptr<ImuMsg> Ptr;
  typedef boost::shared_ptr<const ImuMsg> ConstPtr;
};
  
}