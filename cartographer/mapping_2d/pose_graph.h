/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/trajectory_connectivity_state.h"
#include "cartographer/mapping_2d/pose_graph/constraint_builder.h"
#include "cartographer/mapping_2d/pose_graph/optimization_problem.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_2d {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
class PoseGraph : public mapping::PoseGraph 
{
public:
	PoseGraph(const mapping::proto::PoseGraphOptions& options,
				common::ThreadPool* thread_pool);
	~PoseGraph() override;

	PoseGraph(const PoseGraph&) = delete;
	PoseGraph& operator=(const PoseGraph&) = delete;

	// Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
	// determined by scan matching against 'insertion_submaps.front()' and the
	// node data was inserted into the 'insertion_submaps'. If
	// 'insertion_submaps.front().finished()' is 'true', data was inserted into
	// this submap for the last time.
	mapping::NodeId AddNode(
		std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
		int trajectory_id,
		const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
		EXCLUDES(mutex_);

	// 其实就是将传感器数据转交给 optimization_problem_ 来处理
	// optimization_problem_ 本身不是异步的（内部没有事件队列，但是这里将转交传感器动作本身放进了事件队列）
	// 所以其实优化问题的处理同样是异步的，后端完全处于一个异步状态
	void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override EXCLUDES(mutex_);
	void AddOdometryData(int trajectory_id, const sensor::OdometryData& odometry_data) override EXCLUDES(mutex_);
	// 这个功能在 mapping 2d 中暂时没有实现，还不能用
	void AddFixedFramePoseData( int trajectory_id, const sensor::FixedFramePoseData& fixed_frame_pose_data)
	{
		LOG(FATAL) << "Not yet implemented for 2D.";
	}
	// 以下两个函数都是从 proto 中读出数据，主要是针对 loadmap 的情况
	void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
							const mapping::proto::Submap& submap) override;
	void AddNodeFromProto(const transform::Rigid3d& global_pose,
							const mapping::proto::Node& node) override;
	void AddNodeToSubmap(const mapping::NodeId& node_id,
							const mapping::SubmapId& submap_id) override;
	void AddSerializedConstraints(
		const std::vector<Constraint>& constraints) override;
	void AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) override;

	// 外部获取内部成员对象
	sensor::MapByTime<sensor::ImuData> 
		GetImuData() override EXCLUDES(mutex_);
	sensor::MapByTime<sensor::OdometryData> 
		GetOdometryData() override EXCLUDES(mutex_);
	mapping::PoseGraph::SubmapData 
		GetSubmapData(const mapping::SubmapId& submap_id) EXCLUDES(mutex_) override;
	std::vector<std::vector<int>> 
		GetConnectedTrajectories() override;
	mapping::MapById<mapping::SubmapId, mapping::PoseGraph::SubmapData>
		GetAllSubmapData() EXCLUDES(mutex_) override;
	transform::Rigid3d 
		GetLocalToGlobalTransform(int trajectory_id) EXCLUDES(mutex_) override;
	mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
		GetTrajectoryNodes() override EXCLUDES(mutex_);
	transform::Rigid3d 
		GetInterpolatedGlobalTrajectoryPose( int trajectory_id, const common::Time time) const REQUIRES(mutex_);
	std::vector<Constraint> constraints() override EXCLUDES(mutex_);

	void FinishTrajectory(int trajectory_id) override;
	void FreezeTrajectory(int trajectory_id) override;
  
	void RunFinalOptimization() override;
  
	// 这个函数是在 map_builder 中调用的，用来设置初始位置
	// 这个初始位置是从配置文件中的 initial_trajectory_pose 读入的
	void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time) override EXCLUDES(mutex_);
	  
private:
	// The current state of the submap in the background threads. When this
	// transitions to kFinished, all nodes are tried to match against this submap.
	// Likewise, all new nodes are matched against submaps which are finished.
	enum class SubmapState { kActive, kFinished };
	struct SubmapData 
	{
		std::shared_ptr<const Submap> submap;

		// IDs of the nodes that were inserted into this map together with
		// constraints for them. They are not to be matched again when this submap
		// becomes 'finished'.
		// 被插入到这个 submap 里的所有 node 的 id
		// 这个 submap 的默认状态是 kActive，如果这个状态被写成 kFinished，则这些 node 不会再被匹配
		std::set<mapping::NodeId> node_ids;

		SubmapState state = SubmapState::kActive;
	};
	
	// Allows querying and manipulating the pose graph by the 'trimmers_'. The
	// 'mutex_' of the pose graph is held while this class is used.
	class TrimmingHandle : public mapping::Trimmable 
	{
	public:
		TrimmingHandle(PoseGraph* parent);
		~TrimmingHandle() override {}

		int num_submaps(int trajectory_id) const override;
		void MarkSubmapAsTrimmed(const mapping::SubmapId& submap_id)
			REQUIRES(parent_->mutex_) override;

	private:
		PoseGraph* const parent_;
	};

	// Handles a new work item.
	void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

	// Adds connectivity and sampler for a trajectory if it does not exist.
	void AddTrajectoryIfNeeded(int trajectory_id) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<mapping::SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a node, and starts scan matching in the background.
  void ComputeConstraintsForNode(
      const mapping::NodeId& node_id,
      std::vector<std::shared_ptr<const Submap>> insertion_submaps,
      bool newly_finished_submap) REQUIRES(mutex_);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const mapping::NodeId& node_id,
                         const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // Adds constraints for older nodes whenever a new submap is finished.
  void ComputeConstraintsForOldNodes(const mapping::SubmapId& submap_id)
      REQUIRES(mutex_);

  // Registers the callback to run the optimization once all constraints have
  // been computed, that will also do all work that queue up in 'work_queue_'.
  void HandleWorkQueue() REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);
  
  void DispatchOptimization() EXCLUDES(mutex_);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const mapping::MapById<mapping::SubmapId, pose_graph::SubmapData>&
          global_submap_poses,
      int trajectory_id) const REQUIRES(mutex_);

  mapping::PoseGraph::SubmapData GetSubmapDataUnderLock(
      const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  common::Time GetLatestNodeTime(const mapping::NodeId& node_id,
                                 const mapping::SubmapId& submap_id) const
      REQUIRES(mutex_);

  // Updates the trajectory connectivity structure with a new constraint.
  void UpdateTrajectoryConnectivity(const Constraint& constraint)
      REQUIRES(mutex_);

	  
	  
	/********************************* PARAMETERS ***********************************/
	const mapping::proto::PoseGraphOptions options_;
	common::Mutex mutex_;

	// If it exists, further work items must be added to this queue, and will be
	// considered later.
	// 这是一个待处理（挂起的）工作队列，会添加到线程池中执行
	std::unique_ptr<std::deque<std::function<void()>>> work_queue_ GUARDED_BY(mutex_);

	// How our various trajectories are related.
	mapping::TrajectoryConnectivityState trajectory_connectivity_state_;

	// We globally localize a fraction of the nodes from each trajectory.
	std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
		global_localization_samplers_ GUARDED_BY(mutex_);

	// Number of nodes added since last loop closure.
	int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

	// Whether the optimization has to be run before more data is added.
	bool run_loop_closure_ GUARDED_BY(mutex_) = false;

	// Current optimization problem.
	// 将后端闭环转化为一个非线性的优化问题
	// 使用ceres solver去解spa cost functor
	pose_graph::OptimizationProblem optimization_problem_;
	// 在这个后端的优化问题中引入了 fast correlative scan match 的算法（ real time correlative scan match 论文中的算法）
	// 同时用到了分枝定界（Bound and Branch）
	pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
	std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

	// Submaps get assigned an ID and state as soon as they are seen, even
	// before they take part in the background computations.
	// 这里保存了所有的 submap，包括所有的 trajectory 里的 submap
	mapping::MapById<mapping::SubmapId, SubmapData> submap_data_ GUARDED_BY(mutex_);
	// Global submap poses currently used for displaying data.
	mapping::MapById<mapping::SubmapId, pose_graph::SubmapData> global_submap_poses_ GUARDED_BY(mutex_);

  // Data that are currently being shown.
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNode> trajectory_nodes_ GUARDED_BY(mutex_);
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

	// List of all trimmers to consult when optimizations finish.
	// PoseGraphTrimmer 类是一个纯虚类
	// 真正传进来的 trimmer 应该是 PureLocalizationTrimmer 的对象
	std::vector<std::unique_ptr<mapping::PoseGraphTrimmer>> trimmers_ GUARDED_BY(mutex_);

	// Set of all frozen trajectories not being optimized.
	std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

	// Set of all initial trajectory poses.
	std::map<int, InitialTrajectoryPose> initial_trajectory_poses_ GUARDED_BY(mutex_);
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_H_
