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

#ifndef CARTOGRAPHER_MAPPING_2D_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_2D_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_2d/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter 
{
public:
	explicit MotionFilter(const proto::MotionFilterOptions& options);

	// If the accumulated motion (linear, rotational, or time) is above the
	// threshold, returns false. Otherwise the relative motion is accumulated and
	// true is returned.
	bool IsSimilar(common::Time time, const transform::Rigid3d& pose);
	
	/*
		给的数据的坡度过大的情况我们也要考虑在 filter 中
		这个我们需要首先记录下一个平均的初始坡度
		再在每次的数据进来时进行比对，如果超过的一定范围则返回 true
		否则返回 false
	*/
	bool IsLargeSlope( common::Time time, const Eigen::Quaterniond& gravity_alignment );

private:
	/*
		// Threshold above which range data is inserted based on time.
		double max_time_seconds = 1;
		// Threshold above which range data is inserted based on linear motion.
		double max_distance_meters = 2;
		// Threshold above which range data is inserted based on rotational motion.
		double max_angle_radians = 3;
	 */
	const proto::MotionFilterOptions options_;
	int32_t num_total_ = 0;
	int32_t num_different_ = 0;
	
	int32_t num_total_for_slope_ = 0;
	int32_t num_accepted_slope_ = 0;
	
	common::Time last_time_;
	transform::Rigid3d last_pose_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MOTION_FILTER_H_
