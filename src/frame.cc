#include "frame.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <iomanip>

namespace static_map 
{

using std::cout;
using std::endl;
  
Frame::Frame() : fixed(false) 
{
}

Frame::~Frame()
{
}

bool SortEdge (OutgoingEdge a, OutgoingEdge b) { return (a.weight<b.weight); }

void Frame::computePoseNeighboursKnn( 
  std::vector< std::shared_ptr<Frame> >* frames, int src_id, int k)
{
  neighbours.clear();

  Eigen::Isometry3d& P1 = pose;
  for (int j = 0; j < (*frames).size(); ++j)
  {
    if(src_id == j) 
      continue;
    Eigen::Isometry3d& P2 = (*frames)[j]->pose;

    //Translation
    float diff_tra=(P1.translation()-P2.translation()).norm();
    neighbours.push_back({j,diff_tra});
  }

  if(neighbours.size() < k)
    std::sort(neighbours.begin(),neighbours.end(),SortEdge);
  else{
    std::partial_sort (neighbours.begin(), neighbours.begin()+k, neighbours.end(),SortEdge);
    neighbours.resize(k);
  }
}

void Frame::computeClosestPointsToNeighbours( 
  std::vector< std::shared_ptr<Frame> >* frames, 
  float threshold)
{
  if(this->fixed) 
    return;
  
  Frame& src_frame = *this;

  for (int j = 0; j < src_frame.neighbours.size(); ++j) 
  {
    int dst_id = src_frame.neighbours[j].neighbourIdx;
    src_frame.neighbours[j].correspondances.clear();

    Frame& dst_frame = *frames->at(dst_id);

    Eigen::Vector3d preTra = Eigen::Vector3d(dst_frame.pose.translation());
    auto preInvRot = dst_frame.pose.linear().inverse();
    std::vector<double> dists;

    for (int k = 0; k < src_frame.point_cloud_->size(); ++k) 
    {
      Eigen::Vector3d srcPtOrig = ToEigenVector( (*src_frame.point_cloud_)[k] );
      Eigen::Vector3d srcPtInGlobalFrame = src_frame.pose * srcPtOrig;

      size_t idxMin = 0;
      Eigen::Vector3d srcPtinDstFrame = preInvRot * (srcPtInGlobalFrame-preTra);
      
      pcl::PointXYZ point( srcPtinDstFrame.x(), srcPtinDstFrame.y(), srcPtinDstFrame.z());
      double pointDistSquared = dst_frame.getClosestPoint(point,idxMin);

      if(pointDistSquared < threshold){
        src_frame.neighbours[j].correspondances.push_back({k, (int)idxMin, pointDistSquared});
        dists.push_back(pointDistSquared);
      }
    }
      
////        // this sets iterator middle to the median element
    std::vector<double>::iterator middle = dists.begin() + (dists.size() / 2);
    std::nth_element(dists.begin(), middle, dists.end());
    double nthValue = *middle;

    cout<<"median dist to: "<<dst_id<<" is: "<<nthValue<<endl;
    src_frame.neighbours[j].weight = nthValue * 1.5;
  }
}

double Frame::getClosestPoint(const pcl::PointXYZ& search_point, size_t& ret_index)
{
  if( !pcl_kdtree_ )
  {
    pcl_kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
    pcl_kdtree_->setInputCloud( point_cloud_ );
  }
  
  // do a knn search
  const size_t num_results = 1;
  double distance = 1000.;
  std::vector<int> pointIdxNKNSearch(num_results);
  std::vector<float> pointNKNSquaredDistance(num_results);
  
  if ( pcl_kdtree_->nearestKSearch (search_point, num_results, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    ret_index = pointIdxNKNSearch[0];
    distance = pointNKNSquaredDistance[0];
  }
  
  return distance;
}

}