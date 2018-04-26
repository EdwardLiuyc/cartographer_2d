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

#include "cartographer/mapping_2d/pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

PoseGraph::PoseGraph(const mapping::proto::PoseGraphOptions& options,
                     common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options()),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

PoseGraph::~PoseGraph() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(work_queue_ == nullptr);
}

// 将传入的 insertion_submaps 加入到优化问题中，返回对应的 id
std::vector<mapping::SubmapId> PoseGraph::InitializeGlobalSubmapPoses(
	const int trajectory_id, 
	const common::Time time,
	const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) 
{
	CHECK(!insertion_submaps.empty());
	const auto& submap_data = optimization_problem_.submap_data();
	// 如果目前只有一个 submap，则将这唯一的 submap 加入到优化问题中
	// 并返回这个插入的 submap id （单个元素的 vector）
	if (insertion_submaps.size() == 1) 
	{
		// 首先明确的是，optimization_problem_ 里面所有的 submap data 的位姿全是相对 global 的位姿，而不是local pose
		// 当前只有一个 submap 时，且 optimization_problem_ 里面的 submap data 为空，即还没有开始进行
		
		
		// If we don't already have an entry for the first submap, add one.
		// 同时这个目标 trajectory 里一个 submap 都没有
		if ( submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ) 
		{
			if (initial_trajectory_poses_.count(trajectory_id) > 0) 
			{
				trajectory_connectivity_state_.Connect(
					trajectory_id,
					initial_trajectory_poses_.at(trajectory_id).to_trajectory_id, time);
			}
			optimization_problem_.AddSubmap(
				trajectory_id,
				transform::Project2D(
					ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id) * insertion_submaps[0]->local_pose()));
		}
		
		CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
		const mapping::SubmapId submap_id{trajectory_id, 0};
		CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
		return {submap_id};
	}
  
	// 如果不是上面的情况，则 insertion_submap 中一定要有两个 submap
	CHECK_EQ(2, insertion_submaps.size());
	// EndOfTrajectory 这个函数其实返回的是下一个 trajectory_id 的第一个元素，而不是当前 trajectory_id 的最后一个
	// 所以后面需要用到 std::prev(end_it)->id 找到真正对应的 trajectory_id 的最后一个 submap id
	const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
	CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
	
	const mapping::SubmapId last_submap_id_in_this_trajectory = std::prev(end_it)->id;
	
	// 这里仍然只有两种情况
	// insertion_submaps 中的两个 submap 有可能会全在优化问题中，也有可能只有前面那个在优化问题中
	if ( submap_data_.at(last_submap_id_in_this_trajectory).submap == insertion_submaps.front() ) 
	{
		// In this case, 'last_submap_id_in_this_trajectory' is the ID of 'insertions_submaps.front()'
		// and 'insertions_submaps.back()' is new.
		const auto& first_submap_pose = submap_data.at(last_submap_id_in_this_trajectory).global_pose;
		optimization_problem_.AddSubmap(
			trajectory_id,
			first_submap_pose *
				pose_graph::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
				pose_graph::ComputeSubmapPose(*insertion_submaps[1]));
		return { last_submap_id_in_this_trajectory,
				mapping::SubmapId{trajectory_id, 
				last_submap_id_in_this_trajectory.submap_index + 1}};
	}
	
	CHECK( submap_data_.at(last_submap_id_in_this_trajectory).submap == insertion_submaps.back() );
	const mapping::SubmapId front_submap_id{trajectory_id,last_submap_id_in_this_trajectory.submap_index - 1};
	CHECK( submap_data_.at(front_submap_id).submap == insertion_submaps.front() );
	return { front_submap_id, last_submap_id_in_this_trajectory };
}

/*
 * 这个函数是前段与后端之间的接口！！
 * 前端（local_trajectory_builder）处理了scan data，得到一个 match result（包含insert result）
 * 将这个 insert result 里的 insertion_submaps 和 constant_data 传进函数
 */
mapping::NodeId PoseGraph::AddNode(
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) 
{
	/*
	 * struct Data 
	 * {
			common::Time time;
			// Transform to approximately gravity align the tracking frame as
			// determined by local SLAM.
			Eigen::Quaterniond gravity_alignment;
			// Used for loop closure in 2D: voxel filtered returns in the
			// 'gravity_alignment' frame.
			sensor::PointCloud filtered_gravity_aligned_point_cloud;
			// 这里省略了 3d 的信息
			// The node pose in the local SLAM frame.
			transform::Rigid3d local_pose;
		};
	*/
	const transform::Rigid3d optimized_pose(
		GetLocalToGlobalTransform( trajectory_id ) * constant_data->local_pose);

	common::MutexLocker locker(&mutex_);
	AddTrajectoryIfNeeded(trajectory_id);
	const mapping::NodeId node_id = trajectory_nodes_.Append(
		trajectory_id, mapping::TrajectoryNode{constant_data, optimized_pose});
	
	++num_trajectory_nodes_;

	// Test if the 'insertion_submap.back()' is one we never saw before.
	// std::prev 
	// 两个传参，第一个为 it（迭代器），第二个为 int（n）， 返回这个迭代器的第n个前驱，n默认为1
	// 如果在指定的 trajectory 中一个 submap 都没有
	// 或者指定 trajectory 中有 submap 的情况，但是这个 trajectory 的第一个 submap 不等于 insert 的最后一个
	// 就需要新建一个 submap data 并指向 insert 的最后一个 sumbap
	if ( submap_data_.SizeOfTrajectoryOrZero( trajectory_id ) == 0 
		|| std::prev(submap_data_.EndOfTrajectory(trajectory_id))->data.submap != insertion_submaps.back()) 
	{
		// We grow 'submap_data_' as needed. This code assumes that the first
		// time we see a new submap is as 'insertion_submaps.back()'.
		const mapping::SubmapId submap_id = submap_data_.Append(trajectory_id, SubmapData());
		submap_data_.at(submap_id).submap = insertion_submaps.back();
	}

	// We have to check this here, because it might have changed by the time we
	// execute the lambda.
	const bool newly_finished_submap = insertion_submaps.front()->finished();
	AddWorkItem([=]() REQUIRES(mutex_) 
		{
			ComputeConstraintsForNode( node_id, insertion_submaps, newly_finished_submap );
		});
	return node_id;
}

void PoseGraph::AddWorkItem(const std::function<void()>& work_item) 
{
	// 如果没有 work 队列则直接执行这个任务，否则加入到任务队列中等待执行
	if (work_queue_ == nullptr) 
		work_item();
	else 
		work_queue_->push_back(work_item);
}

// 在需要的情况下添加 trajectory
// 比如在 loadmap 之后， trajectory 0 已经被载入的地图占用
// 这时当前的 trajectory 为 1
void PoseGraph::AddTrajectoryIfNeeded(const int trajectory_id) 
{
	trajectory_connectivity_state_.Add(trajectory_id);
	// Make sure we have a sampler for this trajectory.
	if (!global_localization_samplers_[trajectory_id]) 
	{
		global_localization_samplers_[trajectory_id] = 
			common::make_unique<common::FixedRatioSampler>(options_.global_sampling_ratio());
	}
}

void PoseGraph::AddImuData(const int trajectory_id,
                           const sensor::ImuData& imu_data) 
{
	common::MutexLocker locker(&mutex_);
	AddWorkItem([=]() REQUIRES(mutex_) 
	{
		optimization_problem_.AddImuData(trajectory_id, imu_data);
	});
}

void PoseGraph::AddOdometryData(const int trajectory_id,
                                const sensor::OdometryData& odometry_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddOdometryData(trajectory_id, odometry_data);
  });
}

void PoseGraph::ComputeConstraint(const mapping::NodeId& node_id,
                                  const mapping::SubmapId& submap_id) 
{
	CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

	const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
	const common::Time last_connection_time =
		trajectory_connectivity_state_.LastConnectionTime( node_id.trajectory_id, submap_id.trajectory_id );

// 	LOG(INFO) << "node: " << node_id.trajectory_id << "," << node_id.node_index
// 		<< " submap" << submap_id.trajectory_id << "," << submap_id.submap_index << " time: " << last_connection_time;
	
	/* 
	 * 两种情况只需要 AddConstraint，而不需要 AddGlobalConstraint
	 * 1. node 和 submap 处于同一个 trajectory 中，这时完全需要相信之前的计算结果，不需要全局搜索 TODO(edward)
	 * 2. 在两个不同的 trajectory 间匹配通常是隔一段时间 connect 一次，在这段时间内都是小范围搜索
	*/
	if ( node_id.trajectory_id == submap_id.trajectory_id
		|| node_time < last_connection_time + common::FromSeconds(options_.global_constraint_search_after_n_seconds())) 
	{
		// If the node and the submap belong to the same trajectory or if there has
		// been a recent global constraint that ties that node's trajectory to the
		// submap's trajectory, it suffices to do a match constrained to a local
		// search window.
		const transform::Rigid2d initial_relative_pose =
			optimization_problem_.submap_data().at(submap_id).global_pose.inverse() *
			optimization_problem_.node_data().at(node_id).pose;
		
		constraint_builder_.MaybeAddConstraint(
										submap_id, 
										submap_data_.at(submap_id).submap.get(), 
										node_id,
										trajectory_nodes_.at(node_id).constant_data.get(),
										initial_relative_pose);
	} 
	else if ( global_localization_samplers_[node_id.trajectory_id]->Pulse() ) 
	{
		/*
			默认的参数设定是 
			global_sampling_ratio = 0.003
			global_constraint_search_after_n_seconds = 10
		*/
		constraint_builder_.MaybeAddGlobalConstraint(
										submap_id, 
										submap_data_.at(submap_id).submap.get(), 
										node_id,
										trajectory_nodes_.at(node_id).constant_data.get());
	}
}

void PoseGraph::ComputeConstraintsForOldNodes(const mapping::SubmapId& submap_id) 
{
	const auto& submap_data = submap_data_.at(submap_id);
	for (const auto& node_id_data : optimization_problem_.node_data()) 
	{
		const mapping::NodeId& node_id = node_id_data.id;
		if (submap_data.node_ids.count(node_id) == 0) 
		{
			ComputeConstraint(node_id, submap_id);
		}
	}
}

void PoseGraph::ComputeConstraintsForNode(
    const mapping::NodeId& node_id,
    std::vector<std::shared_ptr<const Submap>> insertion_submaps,
    const bool newly_finished_submap) 
{
	const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
	const std::vector<mapping::SubmapId> submap_ids = InitializeGlobalSubmapPoses(
		node_id.trajectory_id, constant_data->time, insertion_submaps);
	
	CHECK_EQ(submap_ids.size(), insertion_submaps.size());
	
	const mapping::SubmapId matching_id = submap_ids.front();
	const transform::Rigid2d pose = transform::Project2D(
		constant_data->local_pose * transform::Rigid3d::Rotation(constant_data->gravity_alignment.inverse()));
	
	// global_pose * submap_pose * local_pose(local_pose * gravity_alignment)
	// 得到的是这个 node 的 global pose
	const transform::Rigid2d optimized_pose =
		optimization_problem_.submap_data().at(matching_id).global_pose *
		pose_graph::ComputeSubmapPose(*insertion_submaps.front()).inverse() * pose;
	
	// 将计算出的 global pose 加入到优化问题中
	optimization_problem_.AddTrajectoryNode(matching_id.trajectory_id, 
											constant_data->time, 
											pose, 
											optimized_pose,
											constant_data->gravity_alignment);
	
	// 计算两个 submap （或者 1 个）对应的constraint，添加到 constraints_ 中
	for (size_t i = 0; i < insertion_submaps.size(); ++i) 
	{
		const mapping::SubmapId submap_id = submap_ids[i];
		// Even if this was the last node added to 'submap_id', the submap will only
		// be marked as finished in 'submap_data_' further below.
		CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
		submap_data_.at(submap_id).node_ids.emplace(node_id);
		const transform::Rigid2d constraint_transform = 
			pose_graph::ComputeSubmapPose(*insertion_submaps[i]).inverse() * pose;
		constraints_.push_back(Constraint{	submap_id,
											node_id,
											{
												transform::Embed3D(constraint_transform),
												options_.matcher_translation_weight(),
												options_.matcher_rotation_weight()
												
											},
											Constraint::INTRA_SUBMAP
										});
	}

	for ( const auto& submap_id_data : submap_data_ ) 
	{
		if (submap_id_data.data.state == SubmapState::kFinished) 
		{
			CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
			ComputeConstraint(node_id, submap_id_data.id);
		}
	}

	if ( newly_finished_submap ) 
	{
		const mapping::SubmapId finished_submap_id = submap_ids.front();
		SubmapData& finished_submap_data = submap_data_.at(finished_submap_id);
		
		// 在 submap 类里面，有一个标志状态的变量 finished_
		// 对应这里的 submap_data 中的 state （kFinished）
		CHECK(finished_submap_data.state == SubmapState::kActive);
		finished_submap_data.state = SubmapState::kFinished;
		// We have a new completed submap, so we look into adding constraints for
		// old nodes.
		ComputeConstraintsForOldNodes(finished_submap_id);
	}
	
	// 通知 constraint_builder_, 针对这个 node 的计算已经结束了
	// 后面的计算是全局优化的部分，与 constraint_builder_ 无关
	constraint_builder_.NotifyEndOfNode();
	
	++num_nodes_since_last_loop_closure_;
	if ( options_.optimize_every_n_nodes() > 0 &&
		num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) 
	{
		CHECK(!run_loop_closure_);
		DispatchOptimization();
	}
}

void PoseGraph::DispatchOptimization()
{
	run_loop_closure_ = true;
	// If there is a 'work_queue_' already, some other thread will take care.
	if ( work_queue_ == nullptr ) 
	{
		work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
		HandleWorkQueue();
	}
}

common::Time PoseGraph::GetLatestNodeTime(
    const mapping::NodeId& node_id, const mapping::SubmapId& submap_id) const 
{
	common::Time time = trajectory_nodes_.at(node_id).constant_data->time;
	const SubmapData& submap_data = submap_data_.at(submap_id);
	
	if (!submap_data.node_ids.empty()) 
	{
		const mapping::NodeId last_submap_node_id = *submap_data_.at(submap_id).node_ids.rbegin();
		time = std::max(time, trajectory_nodes_.at(last_submap_node_id).constant_data->time);
	}
	return time;
}

void PoseGraph::UpdateTrajectoryConnectivity(const Constraint& constraint) 
{
	CHECK_EQ(constraint.tag, mapping::PoseGraph::Constraint::INTER_SUBMAP);
	const common::Time time = GetLatestNodeTime(constraint.node_id, constraint.submap_id);
				
	trajectory_connectivity_state_.Connect(	constraint.node_id.trajectory_id,
											constraint.submap_id.trajectory_id, 
											time);
}

void PoseGraph::HandleWorkQueue() 
{
	// 一旦所有的 constraint 都计算完了，就会执行传进去的 callback
// 	LOG(INFO) << "Handle word queue, but not right now!";
	constraint_builder_.WhenDone(
		[this](const pose_graph::ConstraintBuilder::Result& result) 
		{
			{
				common::MutexLocker locker(&mutex_);
				constraints_.insert(constraints_.end(), result.begin(), result.end());
			}
			RunOptimization();

			common::MutexLocker locker(&mutex_);
			for (const Constraint& constraint : result) 
			{
				UpdateTrajectoryConnectivity(constraint);
			}
			TrimmingHandle trimming_handle(this);
			for (auto& trimmer : trimmers_) 
			{
				trimmer->Trim(&trimming_handle);
			}

			num_nodes_since_last_loop_closure_ = 0;
			run_loop_closure_ = false;
			while (!run_loop_closure_) 
			{
				// 只要 work_queue 里还有任务没有完成就会一直循环的从第一个任务开始进行
				// 直到整个 work_queue 全空，则停止，直接退出
				if (work_queue_->empty()) 
				{
					work_queue_.reset();
					return;
				}
				work_queue_->front()();
				work_queue_->pop_front();
			}
			
			LOG(INFO) << "Remaining work items in queue: " << work_queue_->size();
			// We have to optimize again.
			HandleWorkQueue();
// 			LOG(INFO) << "seems like it will never be printed!";
		});
}

// 调用场景
// 运行最终的优化或者析构是需要等待所有的计算算完！
void PoseGraph::WaitForAllComputations() 
{
	bool notification = false;
	common::MutexLocker locker(&mutex_);
	const int num_finished_nodes_at_start = constraint_builder_.GetNumFinishedNodes();
	while (!locker.AwaitWithTimeout(
		[this]() REQUIRES(mutex_) 
		{
			return constraint_builder_.GetNumFinishedNodes() == num_trajectory_nodes_;
		},
		common::FromSeconds(1.))) 
	{
		std::ostringstream progress_info;
		progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
					<< 100. *
							(constraint_builder_.GetNumFinishedNodes() -
							num_finished_nodes_at_start) /
							(num_trajectory_nodes_ - num_finished_nodes_at_start)
					<< "%...";
		std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
	}
	std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
	constraint_builder_.WhenDone(
		[this, &notification](const pose_graph::ConstraintBuilder::Result& result) 
		{
			common::MutexLocker locker(&mutex_);
			constraints_.insert(constraints_.end(), result.begin(), result.end());
			notification = true;
		});
	
	locker.Await([&notification]() { return notification; });
}

void PoseGraph::FinishTrajectory(const int trajectory_id) {
  // TODO(jihoonl): Add a logic to notify trimmers to finish the given
  // trajectory.
}

/*
 这个函数是在 loadmap 时调用的，也仅在此情况下调用
 目的是 freeze 载入进来的 trajectory，并在计算时讲 load 进来的 trajectory 作为 global(看 RunOptimization)
 */
void PoseGraph::FreezeTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  trajectory_connectivity_state_.Add(trajectory_id);
  
  got_first_local_to_global_ = false;
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

void PoseGraph::AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                                   const mapping::proto::Submap& submap) 
{
	if (!submap.has_submap_2d())
		return;

	const mapping::SubmapId submap_id = {submap.submap_id().trajectory_id(),
										submap.submap_id().submap_index()};
	std::shared_ptr<const Submap> submap_ptr = std::make_shared<const Submap>(submap.submap_2d());
	const transform::Rigid2d global_submap_pose_2d = transform::Project2D(global_submap_pose);

	common::MutexLocker locker(&mutex_);
	AddTrajectoryIfNeeded(submap_id.trajectory_id);
	
	submap_data_.Insert(submap_id, SubmapData());
	submap_data_.at(submap_id).submap = submap_ptr;
	// Immediately show the submap at the 'global_submap_pose'.
	global_submap_poses_.Insert(submap_id, pose_graph::SubmapData{global_submap_pose_2d});
	AddWorkItem(
		[this, submap_id, global_submap_pose_2d]() REQUIRES(mutex_) 
		{
			CHECK_EQ(frozen_trajectories_.count(submap_id.trajectory_id), 1);
			
			// 这个函数在 loadmap 函数中被调用，即从文件里读出的地图都会被标识为 kFinished
			submap_data_.at(submap_id).state = SubmapState::kFinished;	
			optimization_problem_.InsertSubmap(submap_id, global_submap_pose_2d);
		});
}

void PoseGraph::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                 const mapping::proto::Node& node) {
  const mapping::NodeId node_id = {node.node_id().trajectory_id(),
                                   node.node_id().node_index()};
  std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data =
      std::make_shared<const mapping::TrajectoryNode::Data>(
          mapping::FromProto(node.node_data()));

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(node_id.trajectory_id);
  trajectory_nodes_.Insert(node_id,
                           mapping::TrajectoryNode{constant_data, global_pose});

  AddWorkItem([this, node_id, global_pose]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(node_id.trajectory_id), 1);
    const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
    const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
        constant_data->gravity_alignment.inverse());
    optimization_problem_.InsertTrajectoryNode(
        node_id, constant_data->time,
        transform::Project2D(constant_data->local_pose *
                             gravity_alignment_inverse),
        transform::Project2D(global_pose * gravity_alignment_inverse),
        constant_data->gravity_alignment);
  });
}

void PoseGraph::AddNodeToSubmap(const mapping::NodeId& node_id,
                                const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, node_id, submap_id]() REQUIRES(mutex_) {
    submap_data_.at(submap_id).node_ids.insert(node_id);
  });
}

void PoseGraph::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, constraints]() REQUIRES(mutex_) {
    for (const auto& constraint : constraints) {
      CHECK(trajectory_nodes_.Contains(constraint.node_id));
      CHECK(submap_data_.Contains(constraint.submap_id));
      CHECK(trajectory_nodes_.at(constraint.node_id).constant_data != nullptr);
      CHECK(submap_data_.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(submap_data_.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          UpdateTrajectoryConnectivity(constraint);
          break;
      }
      const Constraint::Pose pose = {
          constraint.pose.zbar_ij *
              transform::Rigid3d::Rotation(
                  trajectory_nodes_.at(constraint.node_id)
                      .constant_data->gravity_alignment.inverse()),
          constraint.pose.translation_weight, constraint.pose.rotation_weight};
      constraints_.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
  });
}

// 函数调用场景：
// 在参数设定为 pure_location （纯粹的定位）时，会传入一个 PureLocalizationTrimmer 用于对 pose_graph 进行微调
void PoseGraph::AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) 
{
	common::MutexLocker locker(&mutex_);
	// C++11 does not allow us to move a unique_ptr into a lambda.
	mapping::PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
	AddWorkItem(
		[this, trimmer_ptr]() REQUIRES(mutex_) { trimmers_.emplace_back(trimmer_ptr); });
}

// 最后的优化，将整个任务队列中的所有排队的计算任务都执行完
void PoseGraph::RunFinalOptimization() 
{
	// TODO (edward) 新的版本中的这个函数实现差异比较大，可以尝试跟进效果
	// new version -> 目前还有问题，用这个实现在结束时会异常退出
	/*
	{
		common::MutexLocker locker(&mutex_);
		AddWorkItem([this]() REQUIRES(mutex_) 
			{
				optimization_problem_.SetMaxNumIterations(options_.max_num_final_iterations());
				DispatchOptimization();
			});
		AddWorkItem([this]() REQUIRES(mutex_) 
			{
				optimization_problem_.SetMaxNumIterations(
					options_.optimization_problem_options().ceres_solver_options().max_num_iterations());
			});
	}
	WaitForAllComputations();*/
	
	// old (stable) version
	WaitForAllComputations();
	optimization_problem_.SetMaxNumIterations(options_.max_num_final_iterations());
	RunOptimization();
	optimization_problem_.SetMaxNumIterations(options_.optimization_problem_options().ceres_solver_options().max_num_iterations());
}

bool PoseGraph::GlobalPoseJump( const transform::Rigid3d& old_global_to_new_global )
{
	// 如果 load map，frozen_trajectories_里面会有从map里加载进来的 trajectory
	// 没有 load map的情况下不需要考虑 global 跳变的问题
	if( frozen_trajectories_.size() == 0 )
		return false;
	
	// 这里最好能取到设定的 submap 的分辨率
	// 但是 resolution 是trajectory builder的参数，与pose graph无关，这里暂时定为 0.05m
	// const double resolution = 0.05;
	const double distance_min = 0.05 * 1.414; // 0.05 * sqrt(2)
	double old_to_new_distance = old_global_to_new_global.translation().norm();
	if( !got_first_local_to_global_ )
	{
		// 第一次获取到两个路径间的转化的情况，不属于跳变的情况
		if( old_to_new_distance > distance_min )
			got_first_local_to_global_ = true;
		
		return false;
	}
	
	LOG(INFO) << "trasnform:" << old_global_to_new_global;
	const double jump_distance = distance_min * 10.;
	// 这里判断在获得了第一次的位置后，global位置发生了跳变
	if( old_to_new_distance > jump_distance )
	{
		LOG(WARNING) << "pose jumped for " << old_to_new_distance << "m";
		return true;
	}
	return false;
}

void PoseGraph::RunOptimization() 
{
	if (optimization_problem_.submap_data().empty())
		return;

	// No other thread is accessing the optimization_problem_, constraints_ and
	// frozen_trajectories_ when executing the Solve. Solve is time consuming, so
	// not taking the mutex before Solve to avoid blocking foreground processing.
	// 这里是唯一 solve optimization problem 的地方，也就是说只有在 RunOptimization 的时候是会去解全局的位置并更新的
	// 所以如果没有后端，永远无法得到基于地图的定位信息
	optimization_problem_.Solve(constraints_, frozen_trajectories_);
	common::MutexLocker locker(&mutex_);

	const auto& submap_data = optimization_problem_.submap_data();
	const auto& node_data = optimization_problem_.node_data();
	// 每个 trajectory 上的每个 node 的 global pose 都会获得新的值
	bool b_pose_has_jumped = false;
	int  last_trajectory_id = 0;
	for (const int trajectory_id : node_data.trajectory_ids()) 
	{
		last_trajectory_id = trajectory_id;
		for (const auto& node : node_data.trajectory(trajectory_id)) 
		{
			auto& mutable_trajectory_node = trajectory_nodes_.at(node.id);
			mutable_trajectory_node.global_pose =
				transform::Embed3D(node.data.pose) *
				transform::Rigid3d::Rotation(mutable_trajectory_node.constant_data->gravity_alignment);
		}

		// Extrapolate all point cloud poses that were not included in the
		// 'optimization_problem_' yet.
		const auto local_to_new_global = ComputeLocalToGlobalTransform(submap_data, trajectory_id);
		const auto local_to_old_global = ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
		transform::Rigid3d old_global_to_new_global = local_to_new_global * local_to_old_global.inverse();
		
		// 如果发生了跳变，则跳过
		// 重新计算下一次，不把这次的计算结果放进 global pose 里面
		// next time, the global sumbap poses is still the formmer one
		if( GlobalPoseJump( old_global_to_new_global ) )
		{	
			// try 2
			old_global_to_new_global = transform::Rigid3d::Identity();
			b_pose_has_jumped = true;
		}
		
														// 当前 trajectory id 对应的最后一个元素
		const mapping::NodeId last_optimized_node_id = std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
		auto node_it = std::next(trajectory_nodes_.find(last_optimized_node_id));
		for (; node_it != trajectory_nodes_.EndOfTrajectory(trajectory_id);
			++node_it) 
		{
			// last_optimized_node_id 之前的（包括）的所有 node 的 global pose 全部更新 
			auto& mutable_trajectory_node = trajectory_nodes_.at(node_it->id);
			mutable_trajectory_node.global_pose = old_global_to_new_global * mutable_trajectory_node.global_pose;
		}
	}
	
	// RunOptimization 其实就是一个将 local 和 global 的关系更新的过程
	// 这个 submap_data 是 optimization_problem_ 里面的计算结果，是无法直接修改的
	if( !b_pose_has_jumped )
		global_submap_poses_ = submap_data;
	else
	{
		// 有跳变的情况下，需要对 submap_data 最后一个插入的 pose 最特殊的处理
		// TODO
		if( global_submap_poses_.SizeOfTrajectoryOrZero(last_trajectory_id)
			== submap_data.SizeOfTrajectoryOrZero(last_trajectory_id) )
			global_submap_poses_ = submap_data;
		else
		{
			pose_graph::SubmapData last_pose_data;
			for (const auto& submap_id_data : submap_data) 
			{
				if( global_submap_poses_.Contains( submap_id_data.id ) )
				{
					global_submap_poses_.at( submap_id_data.id ) = submap_data.at( submap_id_data.id );
					last_pose_data = submap_data.at( submap_id_data.id );
				}
				else
				{
					global_submap_poses_.Insert(  submap_id_data.id , last_pose_data );
				}
			}
		}
	}
}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
PoseGraph::GetTrajectoryNodes() 
{
	common::MutexLocker locker(&mutex_);
	return trajectory_nodes_;
}

sensor::MapByTime<sensor::ImuData> PoseGraph::GetImuData() {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_.imu_data();
}

sensor::MapByTime<sensor::OdometryData> PoseGraph::GetOdometryData() {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_.odometry_data();
}

std::vector<PoseGraph::Constraint> PoseGraph::constraints() {
  std::vector<Constraint> result;
  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : constraints_) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 trajectory_nodes_.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

void PoseGraph::SetInitialTrajectoryPose(const int from_trajectory_id,
                                         const int to_trajectory_id,
                                         const transform::Rigid3d& pose,
                                         const common::Time time) {
  common::MutexLocker locker(&mutex_);
  initial_trajectory_poses_[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

// 插补出指定时间的位姿信息 global_pose
transform::Rigid3d PoseGraph::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, 
	const common::Time time) const 
{
	CHECK(trajectory_nodes_.SizeOfTrajectoryOrZero(trajectory_id) > 0);
	
	// lower_bound
	// 返回第一个在指定 time 之后的元素的迭代器
	// 如果返回的是EndOfTrajectory(trajectory_id)， 则表示所有的数据的时间都是在 time 之前的
	const auto it = trajectory_nodes_.lower_bound(trajectory_id, time);
	if ( it == trajectory_nodes_.BeginOfTrajectory(trajectory_id)) 
	{
		return trajectory_nodes_.BeginOfTrajectory(trajectory_id)->data.global_pose;
	}
	if (it == trajectory_nodes_.EndOfTrajectory(trajectory_id)) 
	{
		return std::prev(trajectory_nodes_.EndOfTrajectory(trajectory_id))->data.global_pose;
	}
	
	// 如果前面都不成立，则用前一个 node 和 当前 node，根据时间来插补出当前的姿态
	return transform::Interpolate(
				transform::TimestampedTransform{std::prev(it)->data.time(),std::prev(it)->data.global_pose},
				transform::TimestampedTransform{it->data.time(),it->data.global_pose},
				time).transform;
}

transform::Rigid3d PoseGraph::GetLocalToGlobalTransform(const int trajectory_id) 
{
	common::MutexLocker locker(&mutex_);
	return ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
}

std::vector<std::vector<int>> PoseGraph::GetConnectedTrajectories() {
  return trajectory_connectivity_state_.Components();
}

mapping::PoseGraph::SubmapData PoseGraph::GetSubmapData(
    const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

mapping::MapById<mapping::SubmapId, mapping::PoseGraph::SubmapData>
PoseGraph::GetAllSubmapData() {
  common::MutexLocker locker(&mutex_);
  mapping::MapById<mapping::SubmapId, mapping::PoseGraph::SubmapData> submaps;
  for (const auto& submap_id_data : submap_data_) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

/*
 * 传参：
 * 1. 全局的所有的 submap 的姿态
 * 2. trajectory_id
 * 函数功能：
 * 计算从指定 trajectory 到全局的一个变换矩阵(translation,rotation)
 */
transform::Rigid3d PoseGraph::ComputeLocalToGlobalTransform(
    const mapping::MapById<mapping::SubmapId, pose_graph::SubmapData>& global_submap_poses,
    const int trajectory_id) const 
{
	auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
	auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
	// begin 和 end 相同则该 trajectory 有且仅有一个 submap
	if (begin_it == end_it) 
	{
		const auto it = initial_trajectory_poses_.find(trajectory_id);
		// 找到了一个 init pose，则用这个 init pose 的信息
		if (it != initial_trajectory_poses_.end()) 
		{
			// 感觉这里的计算有问题？？？ TODO (edward)
			return GetInterpolatedGlobalTrajectoryPose( it->second.to_trajectory_id,it->second.time ) *
						it->second.relative_pose;
		} 
		else 
			return transform::Rigid3d::Identity();
	}
	
	const mapping::SubmapId last_optimized_submap_id = std::prev(end_it)->id;
	// Accessing 'local_pose' in Submap is okay, since the member is const.
	return transform::Embed3D(global_submap_poses.at(last_optimized_submap_id).global_pose) *
			submap_data_.at(last_optimized_submap_id).submap->local_pose().inverse();
}

mapping::PoseGraph::SubmapData PoseGraph::GetSubmapDataUnderLock(
    const mapping::SubmapId& submap_id) 
{
	const auto it = submap_data_.find(submap_id);
	if (it == submap_data_.end()) {
		return {};
	}
	auto submap = it->data.submap;
	if (global_submap_poses_.Contains(submap_id)) 
	{
		// We already have an optimized pose.
		return {submap, transform::Embed3D(global_submap_poses_.at(submap_id).global_pose)};
	}
	// We have to extrapolate.
	return {submap, ComputeLocalToGlobalTransform(global_submap_poses_,submap_id.trajectory_id) * submap->local_pose()};
}

PoseGraph::TrimmingHandle::TrimmingHandle(PoseGraph* const parent)
    : parent_(parent) {}

int PoseGraph::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_.submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

void PoseGraph::TrimmingHandle::MarkSubmapAsTrimmed(
    const mapping::SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  std::set<mapping::NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }
  // Remove all 'constraints_' related to 'submap_id'.
  std::set<mapping::NodeId> nodes_to_remove;
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (constraint.submap_id == submap_id) {
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
            nodes_to_retain.count(constraint.node_id) == 0) {
          // This node will no longer be INTRA_SUBMAP contrained and has to be
          // removed.
          nodes_to_remove.insert(constraint.node_id);
        }
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }
  // Remove all 'constraints_' related to 'nodes_to_remove'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);
  parent_->submap_data_.Trim(submap_id);
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_.TrimSubmap(submap_id);

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  for (const mapping::NodeId& node_id : nodes_to_remove) {
    parent_->trajectory_nodes_.Trim(node_id);
    parent_->optimization_problem_.TrimTrajectoryNode(node_id);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
