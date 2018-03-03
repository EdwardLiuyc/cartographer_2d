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

#include "cartographer/mapping_2d/local_trajectory_builder.h"

#include <limits>
#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping_2d {

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

sensor::RangeData
LocalTrajectoryBuilder::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const 
{
	const sensor::RangeData cropped =
		sensor::CropRangeData(sensor::TransformRangeData(range_data, transform_to_gravity_aligned_frame),
								options_.min_z(), 
								options_.max_z());
	return sensor::RangeData{
		cropped.origin,
		sensor::VoxelFiltered(cropped.returns, options_.voxel_filter_size()),
		sensor::VoxelFiltered(cropped.misses, options_.voxel_filter_size())};
}

std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::RangeData& gravity_aligned_range_data) {
  std::shared_ptr<const Submap> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }
  if (options_.use_online_correlative_scan_matching()) {
    real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        matching_submap->probability_grid(), &initial_ceres_pose);
  }

  auto pose_observation = common::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      pose_prediction, initial_ceres_pose, filtered_gravity_aligned_point_cloud,
      matching_submap->probability_grid(), pose_observation.get(), &summary);
  return pose_observation;
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>
LocalTrajectoryBuilder::AddRangeData(const common::Time time,
                                     const sensor::TimedRangeData& range_data) 
{
	// Initialize extrapolator now if we do not ever use an IMU.
	// 这里的意义在于：
	// 如果不使用 IMU，这里需要直接初始化一个 Extrapolator 供里程计使用
	// 但如果使用 IMU，这个Extrapolator 不需要在这里初始化，可以在第一个imu数据到来的时候再初始化
	// 所以其实不管怎样都是要初始化这个 Extrapolator 的，可以在构造的时候初始化，传入构造的当前时间即可
	// 这样可以省去无谓的判断和一直调用函数的开销
	// 不过这里还要避免在来第一个imu数据之前就开启 Extrapolator 可能带来的问题
	// TODO(edward)
	if ( !options_.use_imu_data() ) 
		InitializeExtrapolator(time);

	// 必须有预测器的存在才进行后面的运算
	// 不管这个预测器的预测到底有没有借助 imu 或者 odom 的信息
	if (extrapolator_ == nullptr) 
	{
		// Until we've initialized the extrapolator with our first IMU message, we
		// cannot compute the orientation of the rangefinder.
		LOG(INFO) << "Extrapolator not yet initialized.";
		return nullptr;
	}
	
	if ( num_accumulated_ == 0 ) 
	{
		first_pose_estimate_ = extrapolator_->ExtrapolatePose(time).cast<float>();
		accumulated_range_data_ = sensor::RangeData{range_data.origin, {}, {}};
	}

	// TODO(gaschler): Take time delta of individual points into account.
	const transform::Rigid3f tracking_delta =
		first_pose_estimate_.inverse() * extrapolator_->ExtrapolatePose(time).cast<float>();
	// TODO(gaschler): Try to move this common behavior with 3D into helper.
	const Eigen::Vector3f origin_in_first_tracking = tracking_delta * range_data.origin;
		
	// Drop any returns below the minimum range and convert returns beyond the
	// maximum range into misses.
	for ( const Eigen::Vector4f& hit : range_data.returns ) 
	{
		const Eigen::Vector3f hit_in_first_tracking = tracking_delta * hit.head<3>();
		const Eigen::Vector3f delta = hit_in_first_tracking - origin_in_first_tracking;
		const float range = delta.norm();
		if ( range >= options_.min_range() ) 
		{
			if (range <= options_.max_range()) 
			{
				accumulated_range_data_.returns.push_back( hit_in_first_tracking );
			} 
			else 
			{
				accumulated_range_data_.misses.push_back(
					origin_in_first_tracking + options_.missing_data_ray_length() / range * delta);
			}
		}
	}
	++num_accumulated_;

	// 加入对重力方向的估计
	// 目前的默认参数里，num_accumulated_range_data = 1 也就是所有的rangedata都会做如下的操作
	if ( num_accumulated_ >= options_.num_accumulated_range_data() ) 
	{
		num_accumulated_ = 0;
		const transform::Rigid3d gravity_alignment = 
			transform::Rigid3d::Rotation( extrapolator_->EstimateGravityOrientation(time) );
		return AddAccumulatedRangeData(
			time,
			TransformToGravityAlignedFrameAndFilter(
				gravity_alignment.cast<float>() * tracking_delta.inverse(),accumulated_range_data_),
			gravity_alignment);
	}
	return nullptr;
}

// 插入收集到的经重力方向修正的 range data，进行scan match，返回matching result
std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>
LocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment) 
{
	// 这个函数里面现将现有的姿态通过 gravity_alignment 转换到 2d空间里
	// 在2d空间里对这个已经重力方向修正好的 range data 进行scan match
	// 获取一个结果在转换会到3d空间（这里同样要用到 gravity_alignment）
	// 再将这个 pose 传回到 extrapolator_ 供之后的姿态估计使用
	if (gravity_aligned_range_data.returns.empty()) 
	{
		LOG(WARNING) << "Dropped empty horizontal range data.";
		return nullptr;
	}

	// Computes a gravity aligned pose prediction.
	const transform::Rigid3d non_gravity_aligned_pose_prediction =
		extrapolator_->ExtrapolatePose(time);
	const transform::Rigid2d pose_prediction = transform::Project2D(
		non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

	// local map frame <- gravity-aligned frame
	std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
		ScanMatch(time, pose_prediction, gravity_aligned_range_data);
	if ( pose_estimate_2d == nullptr ) 
	{
		LOG(WARNING) << "Scan matching failed.";
		return nullptr;
	}
	const transform::Rigid3d pose_estimate = transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
	extrapolator_->AddPose(time, pose_estimate);

	sensor::RangeData range_data_in_local =
		TransformRangeData(gravity_aligned_range_data, transform::Embed3D(pose_estimate_2d->cast<float>()));
	std::unique_ptr<InsertionResult> insertion_result =
		InsertIntoSubmap( time, 
						range_data_in_local, 
						gravity_aligned_range_data,
						pose_estimate, 
						gravity_alignment.rotation());
	return common::make_unique<MatchingResult>(
		MatchingResult{time, pose_estimate, std::move(range_data_in_local),
						std::move(insertion_result)});
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time, 
	const sensor::RangeData& range_data_in_local,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) 
{
// 	motion_filter_.IsLargeSlope( time, gravity_alignment );
	if ( motion_filter_.IsSimilar(time, pose_estimate) )
		return nullptr;

	// Querying the active submaps must be done here before calling
	// InsertRangeData() since the queried values are valid for next insertion.
	// 这里 insertion_submaps 其实是插入 range data 之前的两个 submap 的指针
	// 在插入 range data 之后，如果 front() 那个 submap finished，就不会再存在于 active_submaps_ 里了
	// 但是还是会在这个周期里面存在于 insertion_submaps 里
	std::vector<std::shared_ptr<const Submap>> insertion_submaps;
	for (const std::shared_ptr<Submap>& submap : active_submaps_.submaps()) 
	{
		insertion_submaps.push_back(submap);
	}
	active_submaps_.InsertRangeData(range_data_in_local);

	sensor::AdaptiveVoxelFilter adaptive_voxel_filter(options_.loop_closure_adaptive_voxel_filter_options());
	const sensor::PointCloud filtered_gravity_aligned_point_cloud =
		adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);

	return common::make_unique<InsertionResult>(InsertionResult{
		std::make_shared<const mapping::TrajectoryNode::Data>(
			mapping::TrajectoryNode::Data{
				time,
				gravity_alignment,
				filtered_gravity_aligned_point_cloud,
				{},  // 'high_resolution_point_cloud' is only used in 3D.
				{},  // 'low_resolution_point_cloud' is only used in 3D.
				{},  // 'rotational_scan_matcher_histogram' is only used in 3D.
				pose_estimate}),
		std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

void LocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
	if (extrapolator_ == nullptr) 
	{
		// Until we've initialized the extrapolator we cannot add odometry data.
		LOG(INFO) << "Extrapolator not yet initialized.";
		return;
	}
	extrapolator_->AddOdometryData(odometry_data);
}

void LocalTrajectoryBuilder::InitializeExtrapolator(const common::Time time) 
{
	if (extrapolator_ != nullptr)
		return;
	
	// We derive velocities from poses which are at least 1 ms apart for numerical
	// stability. Usually poses known to the extrapolator will be further apart
	// in time and thus the last two are used.
	// 在初始化 extrapolator_ 时直接使用了构造函数而并不是 InitializeWithImu
	// 可以避免在imu数据不存在时候的麻烦
	// 初始给了一个标准的pose （ 0,0,0） （1,0,0,0）
	constexpr double kExtrapolationEstimationTimeSec = 0.001;
	extrapolator_ = common::make_unique<mapping::PoseExtrapolator>(
		::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
		options_.imu_gravity_time_constant());
	extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

}  // namespace mapping_2d
}  // namespace cartographer
