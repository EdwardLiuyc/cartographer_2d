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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_STATE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_STATE_H_

#include "cartographer/common/time.h"
#include "cartographer/mapping/connected_components.h"

namespace cartographer {
namespace mapping {

// A class that tracks the connectivity state between trajectories. Compared to
// ConnectedComponents it tracks additionally the last time that a global
// constraint connected to trajectories.
//
// This class is thread-compatible.
/*
	这个类的功能和 connected_components 很相似
	区别在于多了一个时间点 last_connection_time_map_
	代码实现的流程是：
	constraint builder 中有一个 finishcomputation，如果这个计算有结果就会更新这个 connection 的时间戳，也就是这个 last connect time
	这个 map 里保存的是 trajectory 之间的 connect 的时间，包括与自身的connect时间
*/
class TrajectoryConnectivityState 
{
public:
	TrajectoryConnectivityState() {}

	TrajectoryConnectivityState(const TrajectoryConnectivityState&) = delete;
	TrajectoryConnectivityState& operator=(const TrajectoryConnectivityState&) = delete;

	// Add a trajectory which is initially connected to only itself.
	void Add(int trajectory_id);

	// Connect two trajectories. If either trajectory is untracked, it will be
	// tracked. This function is invariant to the order of its arguments. Repeated
	// calls to Connect increment the connectivity count and update the last
	// connected time.
	void Connect(int trajectory_id_a, int trajectory_id_b, common::Time time);

	// Determines if two trajectories have been (transitively) connected. If
	// either trajectory is not being tracked, returns false, except when it is
	// the same trajectory, where it returns true. This function is invariant to
	// the order of its arguments.
	inline bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b)
	{
		return connected_components_.TransitivelyConnected(trajectory_id_a,trajectory_id_b);
	}

	// The trajectory IDs, grouped by connectivity.
	inline std::vector<std::vector<int>> Components()
	{
		return connected_components_.Components();
	}
	
	// Return the last connection count between the two trajectories. If either of
	// the trajectories is untracked or they have never been connected returns the
	// beginning of time.
	common::Time LastConnectionTime(int trajectory_id_a, int trajectory_id_b);

private:
	ConnectedComponents connected_components_;

	// Tracks the last time a direct connection between two trajectories has
	// been added. The exception is when a connection between two trajectories
	// connects two formerly unconnected connected components. In this case all
	// bipartite trajectories entries for these components are updated with the
	// new connection time.
	std::map<std::pair<int, int>, common::Time> last_connection_time_map_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_STATE_H_
