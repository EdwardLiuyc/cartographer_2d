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

#ifndef CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
#define CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_

#include <string>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

// Signals when a sample should be taken from a stream of data to select a
// uniformly distributed fraction of the data.
// 这个类主要用来对传感器数据进行采样，也就是说并非所有的传感器数据都会被使用
// 其最主要作用的是 Pulse 函数，如果这里的ratio的设置小于1.0，则调用Pulse就可以跳过一些传感器数据
// 使用得当可以缓解计算压力，但是可能会影响到建图或者定位的精度
class FixedRatioSampler {
 public:
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();

  FixedRatioSampler(const FixedRatioSampler&) = delete;
  FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

  // Returns true if this pulse should result in an sample.
  bool Pulse();

  // Returns a debug string describing the current ratio of samples to pulses.
  std::string DebugString();

 private:
  // Sampling occurs if the proportion of samples to pulses drops below this
  // number.
  const double ratio_;

  int64 num_pulses_ = 0;
  int64 num_samples_ = 0;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
