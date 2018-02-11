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

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph/proto/optimization_problem_options.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace mapping_2d {
namespace pose_graph {

struct NodeData {
  common::Time time;
  transform::Rigid2d initial_pose;
  transform::Rigid2d pose;
  Eigen::Quaterniond gravity_alignment;
};

struct SubmapData {
  transform::Rigid2d global_pose;
};

// Implements the SPA loop closure method.
class OptimizationProblem 
{
public:
	using Constraint = mapping::PoseGraph::Constraint;

	/*
	 * options!
	 * 
	  	// Scaling parameter for Huber loss function.
		double huber_scale = 1;

		// Scaling parameter for the IMU acceleration term.
		double acceleration_weight = 8;

		// Scaling parameter for the IMU rotation term.
		double rotation_weight = 9;

		// Scaling parameter for translation between consecutive nodes.
		double consecutive_node_translation_weight = 2;

		// Scaling parameter for rotation between consecutive nodes.
		double consecutive_node_rotation_weight = 3;

		// Scaling parameter for the FixedFramePose translation.
		double fixed_frame_pose_translation_weight = 11;

		// Scaling parameter for the FixedFramePose rotation.
		double fixed_frame_pose_rotation_weight = 12;

		// If true, the Ceres solver summary will be logged for every optimization.
		bool log_solver_summary = 5;

		common.proto.CeresSolverOptions ceres_solver_options = 7;
	 */
	explicit OptimizationProblem( const mapping::pose_graph::proto::OptimizationProblemOptions& options);
	~OptimizationProblem();

	OptimizationProblem(const OptimizationProblem&) = delete;
	OptimizationProblem& operator=(const OptimizationProblem&) = delete;

	// 处理传感器数据，包括 imu odom，这里已经没有 laser scan了
	// 也就是说雷达数据已经用来在前端创建 submap，这里只是把submap拼接起来，不再做 scan match
	inline void AddImuData(int trajectory_id, const sensor::ImuData& imu_data)
	{
		imu_data_.Append(trajectory_id, imu_data);
	}
	inline void AddOdometryData(int trajectory_id, const sensor::OdometryData& odometry_data)
	{
		odometry_data_.Append(trajectory_id, odometry_data);
	}
	
	inline void AddSubmap(int trajectory_id, const transform::Rigid2d& global_submap_pose)
	{
		submap_data_.Append(trajectory_id, SubmapData{global_submap_pose});
	}
	
	
	void AddTrajectoryNode(int trajectory_id, 
							common::Time time,
							const transform::Rigid2d& initial_pose,
							const transform::Rigid2d& pose,
							const Eigen::Quaterniond& gravity_alignment);
	void InsertTrajectoryNode(const mapping::NodeId& node_id, 
							common::Time time,
							const transform::Rigid2d& initial_pose,
							const transform::Rigid2d& pose,
							const Eigen::Quaterniond& gravity_alignment);
	void TrimTrajectoryNode(const mapping::NodeId& node_id);
		
	void InsertSubmap(const mapping::SubmapId& submap_id,
						const transform::Rigid2d& global_submap_pose);
	void TrimSubmap(const mapping::SubmapId& submap_id);

	void SetMaxNumIterations(int32 max_num_iterations);

	// Optimizes the global poses.
	void Solve(const std::vector<Constraint>& constraints,
				const std::set<int>& frozen_trajectories);

	inline const mapping::MapById<mapping::NodeId, NodeData>& node_data() const { return node_data_; }
	inline const mapping::MapById<mapping::SubmapId, SubmapData>& submap_data() const { return submap_data_; }
	inline const sensor::MapByTime<sensor::ImuData>& imu_data() const { return imu_data_; }
	inline const sensor::MapByTime<sensor::OdometryData>& odometry_data() const { return odometry_data_; }

private:
	std::unique_ptr<transform::Rigid3d> InterpolateOdometry(
		int trajectory_id, common::Time time) const;
	// Uses odometry if available, otherwise the local SLAM results.
	transform::Rigid3d ComputeRelativePose(
		int trajectory_id, const NodeData& first_node_data,
		const NodeData& second_node_data) const;

	mapping::pose_graph::proto::OptimizationProblemOptions options_;
	mapping::MapById<mapping::NodeId, NodeData> node_data_;
	mapping::MapById<mapping::SubmapId, SubmapData> submap_data_;
	
	// map_by_time 会自己处理传入数据的时间，同时在添加数据的时候需要传 trajectory_id
	sensor::MapByTime<sensor::ImuData> imu_data_;
	sensor::MapByTime<sensor::OdometryData> odometry_data_;
};

}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
