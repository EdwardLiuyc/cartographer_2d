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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
	  cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()}{}

// 这个函数在cartographer整个工程中并没有被使用
// 真正用来初始化一个位姿预测器的还是构造函数
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  auto extrapolator = common::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      common::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

// 获取最新的一个位姿的时间，如果没有位姿保存的话就直接返回最小时间
common::Time PoseExtrapolator::GetLastPoseTime() const 
{
	if (timed_pose_queue_.empty()) 
		return common::Time::min();
	
	return timed_pose_queue_.back().time;
}


common::Time PoseExtrapolator::GetLastExtrapolatedTime() const 
{
	if (!extrapolation_imu_tracker_) 
		return common::Time::min();
	
	return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) 
{
	// 初始化imu tracker，且做一个时间对齐
	if ( imu_tracker_ == nullptr ) 
	{
		common::Time tracker_start = time;
		if ( !imu_data_.empty() ) 
		{
			tracker_start = std::min(tracker_start, imu_data_.front().time);
		}
		imu_tracker_ = common::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
	}
	
	timed_pose_queue_.push_back(TimedPose{time, pose});
	// 这里保证了这个 time pose queue 队列只保存传入时间前一段固定时间的位姿
	// 不可能无限保存下去，否则会导致内存溢出的问题
	// 类似有限的缓存一样保存历史的位姿 pose
	while (timed_pose_queue_.size() > 2 &&
			timed_pose_queue_[1].time <= time - pose_queue_duration_) 
	{
		timed_pose_queue_.pop_front();
	}
	
	UpdateVelocitiesFromPoses();
	AdvanceImuTracker(time, imu_tracker_.get());
	TrimImuData();
	TrimOdometryData();
	
	// 这里这样实现是为了避免什么情况发生？？ 难理解？？ TODO （edward）
	odometry_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
	extrapolation_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
	CHECK(timed_pose_queue_.empty() ||
			odometry_data.time >= timed_pose_queue_.back().time);
	odometry_data_.push_back(odometry_data);
	TrimOdometryData();
	
	if (odometry_data_.size() < 2) {
		return;
	}
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta = common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta = odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(odometry_pose_delta.rotation()) / odometry_time_delta;
	  
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) 
{
	const TimedPose& newest_timed_pose = timed_pose_queue_.back();
	CHECK_GE(time, newest_timed_pose.time);
	if (cached_extrapolated_pose_.time != time) {
		const Eigen::Vector3d translation = ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
		const Eigen::Quaterniond rotation =
			newest_timed_pose.pose.rotation() * ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
		cached_extrapolated_pose_ = TimedPose{time, transform::Rigid3d{translation, rotation}};
	}
	
	return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) 
{
	ImuTracker imu_tracker = *imu_tracker_;
	AdvanceImuTracker(time, &imu_tracker);
	return imu_tracker.orientation();
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() 
{
	if (timed_pose_queue_.size() < 2) 
	{
		// We need two poses to estimate velocities.
		return;
	}
	
	CHECK(!timed_pose_queue_.empty());
	const TimedPose& newest_timed_pose = timed_pose_queue_.back();
	const auto newest_time = newest_timed_pose.time;
	const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
	const auto oldest_time = oldest_timed_pose.time;
	const double queue_delta = common::ToSeconds(newest_time - oldest_time);
	if (queue_delta < 0.001) {  // 1 ms
		LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
					<< queue_delta << " ms";
		return;
	}
	const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
	const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
	linear_velocity_from_poses_ = (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
	angular_velocity_from_poses_ =
		transform::RotationQuaternionToAngleAxisVector(
			oldest_pose.rotation().inverse() * newest_pose.rotation()) / queue_delta;
}

// 修剪 imu data 的整个队列的数据（删减无用的imu数据）
void PoseExtrapolator::TrimImuData() 
{
	// imu 数据出队的条件
	// 1. imu 数据队列保存的数据不少于 2 个
	// 2. 保存的pose数据不为空
	// 3. imu_data 中的第二个数据（index==1）的时间戳比最新的pose小，也就是最早的imu时间戳比最新的pose要早
	//    那最早的imu数据已经没有用处了，直接出队，不断的出队一直到imu_data中只有一个imu数据或者imu的时间戳赶上了pose的数据的时间
	while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
			imu_data_[1].time <= timed_pose_queue_.back().time) 
	{
		imu_data_.pop_front();
	}
}

void PoseExtrapolator::TrimOdometryData() 
{
	while (odometry_data_.size() > 2 
		&& !timed_pose_queue_.empty() 
		&& odometry_data_[1].time <= timed_pose_queue_.back().time) 
	{
		odometry_data_.pop_front();
	}
}

void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const 
{
	CHECK_GE(time, imu_tracker->time());
	// 如果没有 imu 数据，则线速度就数据来自于之前的pose的差分或者是来自于 odometry
	if (imu_data_.empty() || time < imu_data_.front().time) 
	{
		// There is no IMU data until 'time', so we advance the ImuTracker and use
		// the angular velocities from poses and fake gravity to help 2D stability.
		imu_tracker->Advance(time);
		imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
		imu_tracker->AddImuAngularVelocityObservation(
			odometry_data_.size() < 2 ? angular_velocity_from_poses_
									: angular_velocity_from_odometry_);
		return;
	}
	
	// 下面是有imu数据的情况
	if (imu_tracker->time() < imu_data_.front().time) {
		// Advance to the beginning of 'imu_data_'.
		imu_tracker->Advance(imu_data_.front().time);
	}
	
	// 这里最后的lamda是定义了imudata的比较函数
	// lower bound的功能为：
	// 返回指向范围 [first, last) 中首个不小于（即大于或等于） value 的元素的迭代器，或若找不到这种元素则返回 last 。
	// 范围 [first, last) 必须已相对于表达式 element < value 或 comp(element, value) 划分。完全排序的范围满足此判别标准。
	// 这里也就是找到第一个时间晚于 imu_tracker 的时间的imu data
	// 如果没有的话就会返回imu_data.end()
	auto it = std::lower_bound(
		imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
		[](const sensor::ImuData& imu_data, const common::Time& time) {
			return imu_data.time < time;
		});
	
	// 将所有的晚于 imu_tracker 时间的 imu_data 都加入到 imu_tracker 中并进行位姿计算
	while (it != imu_data_.end() && it->time < time) 
	{
		imu_tracker->Advance(it->time);
		imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
		imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
		++it;
	}
	imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

// 这里返回的预测的到目标时间的位移量，不是位置
// 简单来说就是 v × t = s
// 所以这里需要讲返回的数值加到现在的位移量上，才能得到现在pose中的translation
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) 
{
	const TimedPose& newest_timed_pose = timed_pose_queue_.back();
	const double extrapolation_delta = common::ToSeconds(time - newest_timed_pose.time);
	if (odometry_data_.size() < 2) 
	{
		return extrapolation_delta * linear_velocity_from_poses_;
	}
	return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
