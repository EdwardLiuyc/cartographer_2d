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

#include "cartographer/mapping_2d/pose_graph/constraint_builder.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace pose_graph {

ConstraintBuilder::ConstraintBuilder(
    const mapping::pose_graph::proto::ConstraintBuilderOptions& options,
    common::ThreadPool* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      sampler_(options.sampling_ratio()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder::~ConstraintBuilder() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(pending_computations_.size(), 0);
  CHECK_EQ(submap_queued_work_items_.size(), 0);
  CHECK(when_done_ == nullptr);
}

// 这里的 Maybe 在于有 sampler
// 不是每次都会执行 ComputeConstraint 和 FinishComputation
void ConstraintBuilder::MaybeAddConstraint(
    const mapping::SubmapId& submap_id, 
	const Submap* const submap,
    const mapping::NodeId& node_id,
    const mapping::TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) 
{
	if (initial_relative_pose.translation().norm() > options_.max_constraint_distance())
		return;
	
	// 这个 sample 的 ratio 默认为 0.3，并不是 1
	// 这个调小可以降低后端计算的负担，不过不清楚对定位效果的影响 TODO (edward)
	if ( sampler_.Pulse() ) 
	{
		common::MutexLocker locker(&mutex_);
		constraints_.emplace_back();
		auto* const constraint = &constraints_.back();
		++pending_computations_[current_computation_];
		const int current_computation = current_computation_;
		ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
			submap_id, 
			&submap->probability_grid(), 
			[=]() EXCLUDES(mutex_) 
			{
				ComputeConstraint(	submap_id, 
									submap, 
									node_id, 
									false, /* match_full_submap */
									constant_data, 
									initial_relative_pose, 
									constraint);
				FinishComputation(current_computation);
			});
	}
}

void ConstraintBuilder::MaybeAddGlobalConstraint(
    const mapping::SubmapId& submap_id, 
	const Submap* const submap,
    const mapping::NodeId& node_id,
    const mapping::TrajectoryNode::Data* const constant_data) 
{
	common::MutexLocker locker(&mutex_);
	constraints_.emplace_back();
	auto* const constraint = &constraints_.back();
	++pending_computations_[current_computation_];
	const int current_computation = current_computation_;
	ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
		submap_id, 
		&submap->probability_grid(), 
		[=]() EXCLUDES(mutex_) 
		{
			ComputeConstraint(
				submap_id, 
				submap, 
				node_id, 
				true, /* match_full_submap */
				constant_data, 
				transform::Rigid2d::Identity(), 
				constraint);
			FinishComputation(current_computation);
		});
}

void ConstraintBuilder::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  ++current_computation_;
}

// When done 解释为将 constraint builder 当前剩余的所有工作完成后在进行 callback 的工作
// 所以是讲 callback 塞到线程池里，并非直接执行
void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)>& callback) 
{
	common::MutexLocker locker(&mutex_);
	CHECK(when_done_ == nullptr);
	when_done_ = common::make_unique<std::function<void(const Result&)>>(callback);
	++pending_computations_[current_computation_];
	const int current_computation = current_computation_;
	
	thread_pool_->Schedule(
		[this, current_computation] 
		{ 
			FinishComputation(current_computation); 
		});
}

void ConstraintBuilder::ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
    const mapping::SubmapId& submap_id, 
	const ProbabilityGrid* const submap,
    const std::function<void()>& work_item) 
{
	if (submap_scan_matchers_[submap_id].fast_correlative_scan_matcher != nullptr) 
	{
		// 如果这个 submap_id 对应的 scan matcher 存在，则不需要创建，直接讲 work_item 存入
		thread_pool_->Schedule(work_item);
	} 
	else 
	{
		// 如果这个 submap_id 对应的 scan matcher 不存在，则先讲工作挂起（塞到这个submap对应的work队列中）
		// 只有在这个队列中有需要处理的 work 时才会创建相应的 scan matcher
		// 创建 scan matcher 同样是放进执行队列中，由线程池自己分配执行
		submap_queued_work_items_[submap_id].push_back(work_item);
		// 这里曾经尝试将判断条件改为 >=1 但是会导致程序段错误，不明原因，需调查 TODO(edward)
		if ( submap_queued_work_items_[submap_id].size() == 1 )
		{
			thread_pool_->Schedule([=]() { ConstructSubmapScanMatcher(submap_id, submap); });
		}
	}
}

void ConstraintBuilder::ConstructSubmapScanMatcher(
    const mapping::SubmapId& submap_id, const ProbabilityGrid* const submap) 
{
	auto submap_scan_matcher =
		common::make_unique<scan_matching::FastCorrelativeScanMatcher>(
			*submap, options_.fast_correlative_scan_matcher_options());
	common::MutexLocker locker(&mutex_);
	submap_scan_matchers_[submap_id] = {submap, std::move(submap_scan_matcher)};
	
	// 在构建这个submap对应的 scan matcher 之后立马将对应的 work 全部塞进线程池
	for (const std::function<void()>& work_item : submap_queued_work_items_[submap_id]) 
	{
		thread_pool_->Schedule(work_item);
	}
	submap_queued_work_items_.erase(submap_id);
}

const ConstraintBuilder::SubmapScanMatcher*
ConstraintBuilder::GetSubmapScanMatcher(const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  const SubmapScanMatcher* submap_scan_matcher =
      &submap_scan_matchers_[submap_id];
  CHECK(submap_scan_matcher->fast_correlative_scan_matcher != nullptr);
  return submap_scan_matcher;
}

void ConstraintBuilder::ComputeConstraint(
    const mapping::SubmapId& submap_id, 
	const Submap* const submap,
    const mapping::NodeId& node_id, 
	bool match_full_submap,
    const mapping::TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    std::unique_ptr<ConstraintBuilder::Constraint>* constraint) 
{
	const transform::Rigid2d initial_pose = ComputeSubmapPose(*submap) * initial_relative_pose;
	const SubmapScanMatcher* const submap_scan_matcher = GetSubmapScanMatcher(submap_id);

	// The 'constraint_transform' (submap i <- node j) is computed from:
	// - a 'filtered_gravity_aligned_point_cloud' in node j,
	// - the initial guess 'initial_pose' for (map <- node j),
	// - the result 'pose_estimate' of Match() (map <- node j).
	// - the ComputeSubmapPose() (map <- submap i)
	float score = 0.;
	// 这里的位姿估计初始值是由 fast_correlative_scan_matcher 计算得到的
	// 之后输入到 CSM 中做进一步的优化
	transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

	// Compute 'pose_estimate' in three stages:
	// 1. Fast estimate using the fast correlative scan matcher.
	// 2. Prune if the score is too low.
	// 3. Refine.
	if ( match_full_submap ) 
	{
		// Match full submap 是在整个 submap 中的所有点和所有角度上搜索最优的 match
		if (submap_scan_matcher->fast_correlative_scan_matcher->MatchFullSubmap(
				constant_data->filtered_gravity_aligned_point_cloud,
				options_.global_localization_min_score(), &score, &pose_estimate)) 
		{
			CHECK_GT(score, options_.global_localization_min_score());
			CHECK_GE(node_id.trajectory_id, 0);
			CHECK_GE(submap_id.trajectory_id, 0);
		} 
		else 
		{
			return;
		}
	} 
	else 
	{
		if (submap_scan_matcher->fast_correlative_scan_matcher->Match(
				initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
				options_.min_score(), &score, &pose_estimate)) 
		{
			// We've reported a successful local match.
			CHECK_GT(score, options_.min_score());
		} 
		else 
		{
			return;
		}
	}
	
	{
		common::MutexLocker locker(&mutex_);
		score_histogram_.Add(score);
	}

	// Use the CSM estimate as both the initial and previous pose. This has the
	// effect that, in the absence of better information, we prefer the original
	// CSM estimate.
	ceres::Solver::Summary unused_summary;
	ceres_scan_matcher_.Match( 	pose_estimate, 
								pose_estimate,
								constant_data->filtered_gravity_aligned_point_cloud,
								*submap_scan_matcher->probability_grid,
								&pose_estimate, &unused_summary);

	const transform::Rigid2d constraint_transform = ComputeSubmapPose(*submap).inverse() * pose_estimate;
	constraint->reset(new Constraint{submap_id,
									node_id,
									{transform::Embed3D(constraint_transform),
									options_.loop_closure_translation_weight(),
									options_.loop_closure_rotation_weight()},
									Constraint::INTER_SUBMAP});
	
	// 下面用于生成格式化的 LOG 信息
	if (options_.log_matches()) 
	{
		std::ostringstream info;
		info << "Node " << node_id << " with "
			<< constant_data->filtered_gravity_aligned_point_cloud.size()
			<< " points on submap " << submap_id << std::fixed;
		if ( match_full_submap ) 
		{
			info << " matches";
		} 
		else 
		{
			const transform::Rigid2d difference =
				initial_pose.inverse() * pose_estimate;
			info << " differs by translation " << std::setprecision(2)
				<< difference.translation().norm() << " rotation "
				<< std::setprecision(3) << std::abs(difference.normalized_angle());
		}
		info << " with score " << std::setprecision(1) << 100. * score << "%.";
		LOG(INFO) << info.str();
	}
}

void ConstraintBuilder::FinishComputation(const int computation_index) 
{
	Result result;
	std::unique_ptr<std::function<void(const Result&)>> callback;
	
	{
		common::MutexLocker locker(&mutex_);
		if (--pending_computations_[computation_index] == 0) 
		{
			pending_computations_.erase(computation_index);
		}
		if ( pending_computations_.empty() ) 
		{
			CHECK_EQ(submap_queued_work_items_.size(), 0);
			if (when_done_ != nullptr) 
			{
				for (const std::unique_ptr<Constraint>& constraint : constraints_) 
				{
					if (constraint != nullptr) 
					{
						result.push_back(*constraint);
					}
				}
				if (options_.log_matches()) 
				{
					LOG(INFO) << constraints_.size() << " computations resulted in "
								<< result.size() << " additional constraints.";
					LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
				}
				constraints_.clear();
				callback = std::move(when_done_);
				when_done_.reset();
			}
		}
	}
	
	if (callback != nullptr) 
	{
		(*callback)(result);
	}
}

int ConstraintBuilder::GetNumFinishedNodes() 
{
	common::MutexLocker locker(&mutex_);
	if (pending_computations_.empty()) 
	{
		return current_computation_;
	}
	return pending_computations_.begin()->first;
}

void ConstraintBuilder::DeleteScanMatcher(const mapping::SubmapId& submap_id) 
{
	common::MutexLocker locker(&mutex_);
	CHECK(pending_computations_.empty());
	submap_scan_matchers_.erase(submap_id);
}

}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
