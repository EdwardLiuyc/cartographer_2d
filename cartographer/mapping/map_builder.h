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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>
#include <unordered_map>

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping_2d/pose_graph.h"
#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.

// 这个类是所有传感器数据的入口，可以理解为把整个cartographer中需要用到的类（功能）串并联在了一起
// 类里面包含了几乎整个 cartographer slam 过程中需要的类
// thread pool 用来讲所有的工作分开到多个线程中去完成
// trajectory builder 用来生成本地的submap
// pose gragh  用来做后端的闭环（gragh based）
// sensor collator 用来收集传感器数据，并做初级的处理
class MapBuilder : public MapBuilderInterface
{
public:
	 
	// 以下这个定义是在 GlobalTrajectoryBuilderInterface 中的定义
	// 这个回调函数是在构造时传入的
	// 这个函数在每次插入一个 rangedata 的时候调用的
	// 如果这个rangedata被插入到submap中的话就会返回nodeid到最后一个参数中去
	// 相反的，如果这个rangedata的数据被抛弃了，返回nullptr到传参里去
	//
	// 	using LocalSlamResultCallback =
	//       std::function<void(int /* trajectory ID */, common::Time,
	//                          transform::Rigid3d /* local pose estimate */,
	//                          sensor::RangeData /* in local frame */,
	//                          std::unique_ptr<const mapping::NodeId>)>;
	using LocalSlamResultCallback = GlobalTrajectoryBuilderInterface::LocalSlamResultCallback;

	MapBuilder(const proto::MapBuilderOptions& options,
				const LocalSlamResultCallback& local_slam_result_callback);
	~MapBuilder() override;

	MapBuilder(const MapBuilder&) = delete;
	MapBuilder& operator=(const MapBuilder&) = delete;

	// Creates a new trajectory builder and returns its index.
	int AddTrajectoryBuilder(
		const std::unordered_set<std::string>& expected_sensor_ids,
		const proto::TrajectoryBuilderOptions& trajectory_options) override;

	// Creates a new trajectory and returns its index. Querying the trajectory
	// builder for it will return 'nullptr'.
	int AddTrajectoryForDeserialization() override;

	// Returns the TrajectoryBuilder corresponding to the specified
	// 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
	// builder.
	// 这个接口将内部的 trajectory_builder 暴露给外部来使用
	// 外部也主要是使用 trajectory_builder 来接受所有的传感器数据
	mapping::TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id) const override;

	// Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
	// i.e. no further sensor data is expected.
	void FinishTrajectory(int trajectory_id) override;

	// Must only be called if at least one unfinished trajectory exists. Returns
	// the ID of the trajectory that needs more data before the MapBuilder is
	// unblocked.
	int GetBlockingTrajectoryId() const;

	// Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
	// error string on failure, or an empty string on success.
	std::string SubmapToProto(const SubmapId& submap_id,
								proto::SubmapQuery::Response* response) override;

	// Serializes the current state to a proto stream.
	void SerializeState(io::ProtoStreamWriter* writer) override;

	// Loads submaps from a proto stream into a new frozen trajectory.
	// 读入一个现有的地图，这个功能很重要，读入的地图必须是上面这个函数生成的文件
	void LoadMap(io::ProtoStreamReader* reader) override;

	int num_trajectory_builders() const;

	mapping::PoseGraph* pose_graph();
	
	int32_t GetRemainingWorkCount();
	std::vector<std::vector<int>> GetConnectedTrajectories();
	
	bool IsConnected( int32_t trajectory_id_a, int32_t trajectory_id_b );

private:
	const proto::MapBuilderOptions options_;
	// 线程池主要是提供给 back end 的优化来使用的，在构造 pose_graph_2d_ 时有使用到
	common::ThreadPool thread_pool_;

	// 这里本来有个针对 3D 场景的 Pose gragh 3d， 已经被删掉了
	std::unique_ptr<mapping_2d::PoseGraph> pose_graph_2d_;
	mapping::PoseGraph* pose_graph_;

	LocalSlamResultCallback local_slam_result_callback_;

	// 这里sensor collator直接用默认构造函数构造
	// 在 trajectory_builder 的构造时有使用到
	sensor::Collator sensor_collator_;
	std::vector<std::unique_ptr<mapping::TrajectoryBuilder>> trajectory_builders_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
