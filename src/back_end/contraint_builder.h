#pragma once

#include "submap.h"

namespace static_map
{
namespace back_end
{
  
struct Constraint 
{
  struct Pose 
  {
    Eigen::Isometry3d zbar_ij;
    double translation_weight;
    double rotation_weight;
  };

  SubmapId submap_id;  // 'i' in the paper.
  FrameId  frame_id;   // 'j' in the paper.

  // Pose of the node 'j' relative to submap 'i'.
  Pose pose;

  // Differentiates between intra-submap (where node 'j' was inserted into
  // submap 'i') and inter-submap constraints (where node 'j' was not inserted
  // into submap 'i').
  enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
};

class ConstraintBuilder
{
public:
  ConstraintBuilder() = default;
  ~ConstraintBuilder() = default;

  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;
  
  void MaybeAddConstraint(
    const SubmapId& submap_id, const std::vector<Submap>& submaps,
    const FrameId& frame_id,
    const mapping::TrajectoryNode::Data* const constant_data,
    const std::vector<mapping::TrajectoryNode>& submap_nodes,
    const transform::Rigid3d& global_node_pose,
    const transform::Rigid3d& global_submap_pose);
  
private:
  std::deque<std::unique_ptr<Constraint>> constraints_;
  
};



  
} // namespace back_end 
} // namespace static_map