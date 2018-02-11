/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
	
// 当前位姿的预测器
// 维护了一个队列，保存了一段时间的pose，这段时间的长度实在构造或者初始化时给定的
// 同时还要规定 imu 的重力常数，因为这个预测主要是靠imu_tracker实现的
class PoseExtrapolator 
{
public:
	explicit PoseExtrapolator(common::Duration pose_queue_duration,
								double imu_gravity_time_constant);

	PoseExtrapolator(const PoseExtrapolator&) = delete;
	PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

	// unused !!
	static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
		common::Duration pose_queue_duration, double imu_gravity_time_constant,
		const sensor::ImuData& imu_data);

	// Returns the time of the last added pose or Time::min() if no pose was added
	// yet.
	common::Time GetLastPoseTime() const;
	common::Time GetLastExtrapolatedTime() const;

	void AddPose( common::Time time, const transform::Rigid3d& pose );
	void AddImuData(const sensor::ImuData& imu_data);
	void AddOdometryData(const sensor::OdometryData& odometry_data);
	
	transform::Rigid3d ExtrapolatePose(common::Time time);

	// Gravity alignment estimate.
	Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

private:
	void UpdateVelocitiesFromPoses();
	void TrimImuData();
	void TrimOdometryData();
	void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  
	// 分别为估计当前的旋转量和位移量
	// 旋转量用 imu_tracker 来估计，而位移量用线性速度来估计
	// 这个线性速度的来源可能是 odometry，如果没有的话就是cartographer的前段得到的两个pose的偏移量来估计速度
	Eigen::Quaterniond ExtrapolateRotation(common::Time time,
											ImuTracker* imu_tracker) const;
	Eigen::Vector3d ExtrapolateTranslation(common::Time time);

	const common::Duration pose_queue_duration_;
	struct TimedPose 
	{
		common::Time time;
		transform::Rigid3d pose;
	};
	
	const double gravity_time_constant_;
	// imu 数据传入后保存到队列中，并传入到imu_tracker中去估计当前的姿态（orientation）
	// 不参与线速度和角速度计算
	std::deque<sensor::ImuData> imu_data_;
	// 其实从本质上来说，他们用的是同一个imu_tracker的数据
	// 但是这里实现出三个imu_tracker到底是为了代码可读性还是别有深意有待探究 =_=! TODO (edward)
	std::unique_ptr<ImuTracker> imu_tracker_;
	std::unique_ptr<ImuTracker> odometry_imu_tracker_;
	std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;

	// 保存传入的 odom 数据
	std::deque<sensor::OdometryData> odometry_data_;
 	// 在有 odom 数据的情况下，从odom数据里获取的线速度和角速度信息
	Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
		
	// 
	std::deque<TimedPose> timed_pose_queue_;
	TimedPose cached_extrapolated_pose_;
	// 在没有 imu 或者 odom 数据的情况，从历史的poses来估计当前的角速度和线速度
	// 功能在 UpdateVelocitiesFromPoses 函数里
	Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
