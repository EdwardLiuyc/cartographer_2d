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

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_2d {
namespace pose_graph {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
inline transform::Rigid2d ComputeSubmapPose(const Submap& submap)
{
	// 其实并没有做计算，只是获取了 submap 中的位姿信息并将其转成 2d 的输出
	return transform::Project2D(submap.local_pose());
}	

/*
	这里的异步的概念很重要，local map 构建是会有一个对当前位姿的估计，这个是属于 front end 的工作
	而后端则会异步的对前段的结果做优化，这个优化并不需要实时
	如果前段做的足够准，后端甚至不需要的
	Asynchronously computes constraints.
*/
// Intermingle an arbitrary number of calls to MaybeAddConstraint() or
// MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
// done the 'callback' will be called with the result and another
// MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder 
{
public:
	using Constraint = mapping::PoseGraph::Constraint;
	using Result = std::vector<Constraint>;

	/*
	 * 	option 目前有以下这些参数
	 * 
		// A constraint will be added if the proportion of added constraints to
		// potential constraints drops below this number.
		double sampling_ratio = 1;

		// Threshold for poses to be considered near a submap.
		double max_constraint_distance = 2;

		// Threshold for the scan match score below which a match is not considered.
		// Low scores indicate that the scan and map do not look similar.
		double min_score = 4;

		// Threshold below which global localizations are not trusted.
		double global_localization_min_score = 5;

		// Weight used in the optimization problem for the translational component of
		// loop closure constraints.
		double loop_closure_translation_weight = 13;

		// Weight used in the optimization problem for the rotational component of
		// loop closure constraints.
		double loop_closure_rotation_weight = 14;

		// If enabled, logs information of loop-closing constraints for debugging.
		bool log_matches = 8;

		// Options for the internally used scan matchers.
		mapping_2d.scan_matching.proto.FastCorrelativeScanMatcherOptions
			fast_correlative_scan_matcher_options = 9;
		mapping_2d.scan_matching.proto.CeresScanMatcherOptions
			ceres_scan_matcher_options = 11;
	 */
	// 构造用到的线程池是由 pose_graph 传进来的，追根溯源是由 map_builder 创建的
	// 主要用来完成后端闭环优化
	ConstraintBuilder(
		const mapping::pose_graph::proto::ConstraintBuilderOptions& options,
		common::ThreadPool* thread_pool);
	~ConstraintBuilder();

	ConstraintBuilder(const ConstraintBuilder&) = delete;
	ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

	// Schedules exploring a new constraint between 'submap' identified by
	// 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
	// 'initial_relative_pose' is relative to the 'submap'.
	//
	// The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
	// all computations are finished.
	// 以下两个函数都是将 ComputeConstraint 添加到 work 队列中，只是search的方式不同
	// 前者是在一定范围（参数决定）内搜索，后者是在整个 submap 中搜索
	// 这个计算 ceres 会开启一个后台线程来计算，所以需要一个 when done 的回调函数在计算真正完成的时候执行
	void MaybeAddConstraint(
		const mapping::SubmapId& submap_id, 
		const Submap* submap,
		const mapping::NodeId& node_id,
		const mapping::TrajectoryNode::Data* const constant_data,
		const transform::Rigid2d& initial_relative_pose);

	// Schedules exploring a new constraint between 'submap' identified by
	// 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
	// This performs full-submap matching.
	//
	// The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
	// all computations are finished.
	// 比上面函数少一个传参，是因为这里使用的是 full submap
	// initial relative pose 是通过submap计算出来的，无需给定
	void MaybeAddGlobalConstraint(
		const mapping::SubmapId& submap_id, 
		const Submap* submap,
		const mapping::NodeId& node_id,
		const mapping::TrajectoryNode::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const mapping::SubmapId& submap_id);

 private:
	struct SubmapScanMatcher 
	{
		const ProbabilityGrid* probability_grid;
		std::unique_ptr<scan_matching::FastCorrelativeScanMatcher>
			fast_correlative_scan_matcher;
	};

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      const mapping::SubmapId& submap_id, const ProbabilityGrid* submap,
      const std::function<void()>& work_item) REQUIRES(mutex_);

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  void ConstructSubmapScanMatcher(const mapping::SubmapId& submap_id,
                                  const ProbabilityGrid* submap)
      EXCLUDES(mutex_);

  // Returns the scan matcher for a submap, which has to exist.
  const SubmapScanMatcher* GetSubmapScanMatcher(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_);

	// Runs in a background thread and does computations for an additional
	// constraint, assuming 'submap' and 'compressed_point_cloud' do not change
	// anymore. As output, it may create a new Constraint in 'constraint'.
	void ComputeConstraint(
		const mapping::SubmapId& submap_id, 
		const Submap* submap,
		const mapping::NodeId& node_id, 
		bool 	match_full_submap,
		const mapping::TrajectoryNode::Data* const constant_data,
		const transform::Rigid2d& initial_relative_pose,
		std::unique_ptr<Constraint>* constraint) EXCLUDES(mutex_);

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  void FinishComputation(int computation_index) EXCLUDES(mutex_);

	/********************************** MEMBER PARAMETERS ***************************************************/
	// 构造直接传入的参数
	const mapping::pose_graph::proto::ConstraintBuilderOptions options_;
	common::ThreadPool* thread_pool_;
	common::Mutex mutex_;

	// 'callback' set by WhenDone().
	std::unique_ptr<std::function<void(const Result&)>> when_done_ GUARDED_BY(mutex_);

	// Index of the node in reaction to which computations are currently
	// added. This is always the highest node index seen so far, even when older
	// nodes are matched against a new submap.
	int current_computation_ GUARDED_BY(mutex_) = 0;
	
	// For each added node, maps to the number of pending computations that were
	// added for it.
	// 保存了所有被挂起的计算，之所以为异步后端正因为计算不是实时的，来不及计算的会被挂起
	// key(int) -> value(int) ? TODO(edward) 两个int到底对应的是什么意义
	std::map<int, int> pending_computations_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of already constructed scan matchers by 'submap_id'.
  std::map<mapping::SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);

	// Map by 'submap_id' of scan matchers under construction, and the work
	// to do once construction is done.
	std::map<mapping::SubmapId, std::vector<std::function<void()>>>
		submap_queued_work_items_ GUARDED_BY(mutex_);

	common::FixedRatioSampler sampler_;
	scan_matching::CeresScanMatcher ceres_scan_matcher_;

	// Histogram of scan matcher scores.
	// 得分直方图
	common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_CONSTRAINT_BUILDER_H_
