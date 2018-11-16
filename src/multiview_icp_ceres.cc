#include "multiview_icp_ceres.h"

#include <ceres/solver.h>
#include <ceres/problem.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>

#define useLocalParam

namespace static_map
{
namespace back_end
{
namespace ICP_Ceres
{
  
using namespace Eigen;
using std::cout;
using std::endl;

ceres::Solver::Options getOptions(){
  // Set a few options
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 50;

  cout << "Ceres Solver getOptions()" << endl;
  cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
  cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
  cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

  return options;
}

ceres::Solver::Options getOptionsMedium(){
  // Set a few options
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  //If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
  options.use_explicit_schur_complement=true;
  options.max_num_iterations = 50;

  cout << "Ceres Solver getOptionsMedium()" << endl;
  cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
  cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
  cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

  return options;
}

void solve(ceres::Problem &problem, bool smallProblem=false){
  ceres::Solver::Summary summary;
  ceres::Solve(smallProblem ? getOptions() : getOptionsMedium(), &problem, &summary);
  if(!smallProblem) std::cout << "Final report:\n" << summary.FullReport();
}

Isometry3d eigenQuaternionToIso(const Eigen::Quaterniond& q, const Vector3d& t)
{
  Isometry3d poseFinal = Isometry3d::Identity();
  poseFinal.linear() = q.toRotationMatrix();
  poseFinal.translation() = t;
  return poseFinal;
}

Isometry3d pointToPoint_EigenQuaternion( std::vector<Vector3d>&src,
                                         std::vector<Vector3d>&dst)
{
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t(0,0,0);

    ceres::Problem problem;

    for (int i = 0; i < src.size(); ++i) {
      // first viewpoint : dstcloud, fixed
      // second viewpoint: srcCloud, moves
      ceres::CostFunction* cost_function = ICPCostFunctions::PointToPointError_EigenQuaternion::Create(dst[i],src[i]);
      problem.AddResidualBlock(cost_function, NULL, q.coeffs().data(), t.data());
    }

#ifdef useLocalParam
    ceres::LocalParameterization *quaternion_parameterization = new eigen_quaternion::EigenQuaternionParameterization;
    problem.SetParameterization(q.coeffs().data(),quaternion_parameterization);
#endif

    solve(problem);
    return eigenQuaternionToIso(q,t);
}

void ceresOptimizer( std::vector< std::shared_ptr<Frame<pcl::PointXYZ>> >& frames, bool robust)
{
  ceres::Problem problem;
  std::vector<Eigen::Quaterniond> qs(frames.size());
  std::vector<Eigen::Vector3d> ts(frames.size());

  //extract initial camera poses
  for(int i=0; i<frames.size(); i++){
    Isometry3d originalPose = frames[i]->pose.cast<double>();
    Eigen::Quaterniond q;// = Eigen::Map<Eigen::Quaterniond>(cameras+i*7);
    Eigen::Vector3d t;// = Eigen::Map<Eigen::Vector3d>(cameras+i*7+4);

    q = Eigen::Quaterniond(originalPose.linear());
    t = Eigen::Vector3d(originalPose.translation());

    qs[i]=q;
    ts[i]=t;

    if (i==0){
      frames[i]->fixed_=true;
    }
  }

  cout<<"ok ceres"<<endl;

//   eigen_quaternion::EigenQuaternionParameterization *quaternion_parameterization = new eigen_quaternion::EigenQuaternionParameterization;
  
  ceres::LocalParameterization* quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;
  //add edges
  for(int src_id = 0; src_id < frames.size(); src_id++){
    auto& src_frame = *frames[src_id];
    if(src_frame.fixed_) 
      continue;

    Eigen::Quaterniond& srcQ = qs[src_id];
    Eigen::Vector3d& srcT = ts[src_id];
    
    for (int j = 0; j < src_frame.neighbours.size(); ++j) {
      OutgoingEdge& dstEdge = src_frame.neighbours[j];
      auto& dst_frame = *frames.at(dstEdge.neighbourIdx);

      int dst_id = dstEdge.neighbourIdx;

      Eigen::Quaterniond& dstQ = qs[dst_id];
      Eigen::Vector3d& dstT = ts[dst_id]; //dst_frame
      std::cout << "dstEdge.correspondances size :" << dstEdge.correspondances.size() << std::endl;
      for(auto& corr : dstEdge.correspondances){

        // first viewpoint : dstcloud, fixed
        // second viewpoint: srcCloud, moves
        ceres::CostFunction* cost_function = 
          ICPCostFunctions::PointToPointErrorGlobal::Create(
            ToEigenVector( (*dst_frame.point_cloud_)[corr.second]), 
            ToEigenVector( (*dst_frame.point_cloud_)[corr.first]) );

        ceres::LossFunction* loss = NULL;
        if(robust) 
          loss = new ceres::SoftLOneLoss( dstEdge.weight );
        
        problem.AddResidualBlock(cost_function, loss, srcQ.coeffs().data(),
                                 srcT.data(),dstQ.coeffs().data(), dstT.data());
        
        problem.SetParameterization( srcQ.coeffs().data(), quaternion_local_parameterization);
        problem.SetParameterization( dstQ.coeffs().data(), quaternion_local_parameterization);
      }
    }
  }
  
  std::cout << "add edge finished " << std::endl;
  for (int i = 0; i < frames.size(); ++i) {
      if(frames[i]->fixed_){
          std::cout<<i<<" fixed"<<endl;
          problem.SetParameterBlockConstant(qs[i].coeffs().data());
          problem.SetParameterBlockConstant(ts[i].data());
      }
  }

  solve(problem);

  //update frame poses
  for (int i = 0; i < frames.size(); ++i) {
      frames[i]->pose=eigenQuaternionToIso(qs[i],ts[i]);
  }
}

}  // namespace ICP_Ceres
}  // namespace back_end
}  // namespace static_map