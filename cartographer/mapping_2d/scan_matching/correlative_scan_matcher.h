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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"

// 这个文件里的定义是所有 correlative scan matching 的基础
// 这些 scan matching 算法的核心都类似，只是real time 算法里用到了 multi-resolution 的方式加快了搜索的速度
namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan;	// 整型的离散点

// Describes the search space.
// 定义搜索范围参数
// 在类似的 correlative scan matching 的算法中，都是要实现准备好一个 lookup table
// 新的 scan 信息来了之后在旧的 look up table 中寻找最合适的匹配，从而获取 translation 和 rotation
struct SearchParameters 
{
	// Linear search window in pixel offsets; bounds are inclusive.
	// 一个搜索窗口，便捷是包括在内的
	// 单位为像素
	struct LinearBounds 
	{
		int32_t min_x;
		int32_t max_x;
		int32_t min_y;
		int32_t max_y;
	};

	// 构造时需要传入的参数有：
	// 1. 角度和位移的查找窗口大小
	// 2. 点云
	// 3. 分辨率
	SearchParameters(double linear_search_window, double angular_search_window,
					const sensor::PointCloud& point_cloud, double resolution);

	// For testing.
	SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
					double angular_perturbation_step_size, double resolution);

	// Tightens the search window as much as possible.
	void ShrinkToFit(const std::vector<DiscreteScan>& scans,
                   const CellLimits& cell_limits);

  int num_angular_perturbations;
  double angular_perturbation_step_size;
	double resolution;	//< 分辨率，将传入的（double）window的大小转换到像素（单位化）
  int num_scans;
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate 
{
	Candidate(const int init_scan_index, 
				const int init_x_index_offset,
				const int init_y_index_offset,
				const SearchParameters& search_parameters)
			: scan_index(init_scan_index),
			x_index_offset(init_x_index_offset),
			y_index_offset(init_y_index_offset),
			x(-y_index_offset * search_parameters.resolution),
			y(-x_index_offset * search_parameters.resolution),
			orientation((scan_index - search_parameters.num_angular_perturbations) *
						search_parameters.angular_perturbation_step_size) {}

	// Index into the rotated scans vector.
	int scan_index = 0;

	// Linear offset from the initial pose.
	int x_index_offset = 0;
	int y_index_offset = 0;

	// Pose of this Candidate relative to the initial pose.
	// 这个是匹配得出的最终结果， xy轴的偏移以及偏转角
	double x = 0.;
	double y = 0.;
	double orientation = 0.;

	// Score, higher is better.
	float score = 0.f;

	bool operator<(const Candidate& other) const { return score < other.score; }
	bool operator>(const Candidate& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
