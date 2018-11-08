#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace static_map
{

struct Correspondance{
    int first;
    int second;
    double dist;
};

struct OutgoingEdge{
    int neighbourIdx;
    float weight; //==error of correspondances
    std::vector<Correspondance> correspondances;  //srcIdx, dstIdx, dist
    Eigen::Isometry3d P_relative;
};

inline 
Eigen::Vector3d ToEigenVector( const pcl::PointXYZ& point )
{
  Eigen::Vector3d vec;
  vec << point.x, point.y, point.z;
  
  return vec;
}

class Frame
{
public:
    Frame();
    ~Frame();

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    bool fixed;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    std::vector<OutgoingEdge> neighbours;

    void computePoseNeighboursKnn( std::vector< std::shared_ptr<Frame> >* frames, int i, int k);
    void computeClosestPointsToNeighbours( std::vector< std::shared_ptr<Frame> >* frames, float thresh);
    double getClosestPoint(const pcl::PointXYZ& search_point, size_t& ret_index); //query_pt must be in dstFrame

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pcl_kdtree_ = nullptr;

private:

};  

}