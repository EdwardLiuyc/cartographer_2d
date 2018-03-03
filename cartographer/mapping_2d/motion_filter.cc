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

#include "cartographer/mapping_2d/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MotionFilterOptions options;
  options.set_max_time_seconds(
      parameter_dictionary->GetDouble("max_time_seconds"));
  options.set_max_distance_meters(
      parameter_dictionary->GetDouble("max_distance_meters"));
  options.set_max_angle_radians(
      parameter_dictionary->GetDouble("max_angle_radians"));
  return options;
}

MotionFilter::MotionFilter(const proto::MotionFilterOptions& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) 
{
	// 记录一段时间内的 filter 实际采用的 pose 的比例
	LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
		<< "Motion filter reduced the number of nodes to "
		<< 100. * num_different_ / num_total_ << "%.";
		
	++num_total_;
	if ( num_total_ > 1 
		&& time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) 
		&& (pose.translation() - last_pose_.translation()).norm() <= options_.max_distance_meters() 
		&& transform::GetAngle(pose.inverse() * last_pose_) <= options_.max_angle_radians()) 
	{
		return true;
	}
	
	last_time_ = time;
	last_pose_ = pose;
	++num_different_;
	return false;
}

bool MotionFilter::IsLargeSlope( common::Time time, 
								 const Eigen::Quaterniond& gravity_alignment )
{
	LOG_IF_EVERY_N(INFO, num_total_for_slope_ >= 500, 500)
		<< "Motion filter (for slope) reduced the number of nodes to "
		<< 100. * num_accepted_slope_ / num_total_for_slope_ << "%.";
	
	// 这个 pointer vector 反应的是整个平台的姿态 （ roll pitch yaw ），单位是弧度
	Eigen::Vector3d pointer_vector = transform::RotationQuaternionToAngleAxisVector( gravity_alignment );
	// LOG(INFO) << pointer_vector;
	
	++num_total_for_slope_;
	if( num_total_for_slope_ > 1 
		&& ( fabs(pointer_vector[0]) > 0.5 || fabs(pointer_vector[1] > 0.5)) )
	{
		return true;
	}
	
	++num_accepted_slope_;
	return false;
}

}  // namespace mapping_2d
}  // namespace cartographer
