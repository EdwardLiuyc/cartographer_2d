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

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
	
// 用 imu 数据中的角速度和线性加速度来跟踪（track）当前的方向（orientation）
// roll 和 pitch 不会漂移，但是 yaw 是会漂移的
class ImuTracker 
{
public:
	ImuTracker(double imu_gravity_time_constant, common::Time time);

	// Advances to the given 'time' and updates the orientation to reflect this.
	void Advance(common::Time time);

	// Updates from an IMU reading (in the IMU frame).
	void AddImuLinearAccelerationObservation(
		const Eigen::Vector3d& imu_linear_acceleration);
	void AddImuAngularVelocityObservation(
		const Eigen::Vector3d& imu_angular_velocity);
  
	// add by edward liu
	void AddImuOrientationDirectly( common::Time time, 
									const Eigen::Quaterniond& rotation );
	
	// cartographer 本来对于imu的处理很初级，这里当做是第一个模式
	enum Mode
	{
		kAddAccAndAngularVelocity,
		kAddOrientationDirectly,
		kModeCount
	};
	
	void SetTrackMode( Mode track_mode )
	{
		track_mode_ = track_mode;
	}
	
	// Query the current time.
	common::Time time() const { return time_; }
	// Query the current orientation estimate.
	Eigen::Quaterniond orientation() const { return orientation_; }

private:
	const double imu_gravity_time_constant_;
	// time_ 的赋值只有两个函数里有，一个在构造函数里，一个在 Advance 里
	// time_ 是当前 imu 最新的时间
	common::Time time_;
	common::Time last_linear_acceleration_time_;
	Eigen::Quaterniond orientation_;
	Eigen::Vector3d gravity_vector_;
	Eigen::Vector3d imu_angular_velocity_;
	
	Mode track_mode_;
};

}  // namespace mapping
}  // namespace cartographer

/*
	预备知识：
	惯性测量单元（英文：Inertial measurement unit，简称IMU）是测量物体三轴姿态角(或角速率)以及加速度的装置。

	陀螺仪及加速度计是IMU的主要元件，其精度直接影响到惯性系统的精度。
	在实际工作中，由于不可避免的各种干扰因素，而导致陀螺仪及加速度计产生误差，
	从初始对准开始，其导航误差就随时间而增长，尤其是位置误差，这是惯导系统的主要缺点。
	所以需要利用外部信息进行辅助，实现组合导航，使其有效地减小误差随时间积累的问题。
	为了提高可靠性，还可以为每个轴配备更多的传感器。一般而言IMU要安装在被测物体的重心上。
	一般情况，一个IMU包含了三个单轴的加速度计和三个单轴的陀螺仪，
	加速度计检测物体在载体坐标系统独立三轴的加速度信号，而陀螺仪检测载体相对于导航坐标系的角速度信号，
	测量物体在三维空间中的角速度和加速度，并以此解算出物体的姿态。在导航中有着很重要的应用价值。

	IMU大多用在需要进行运动控制的设备，如汽车和机器人上。
	也被用在需要用姿态进行精密位移推算的场合，如潜艇、飞机、导弹和航天器的惯性导航设备等。
	利用三轴地磁解耦和三轴加速度计，受外力加速度影响很大，在运动/振动等环境中，输出方向角误差较大,
	此外地磁传感器有缺点，它的绝对参照物是地磁场的磁力线,地磁的特点是使用范围大，但强度较低，约零点几高斯，
	非常容易受到其它磁体的干扰，如果融合了Z轴陀螺仪的瞬时角度，就可以使系统数据更加稳定。
	加速度测量的是重力方向，在无外力加速度的情况下，能准确输出ROLL/PITCH两轴姿态角度 并且此角度不会有累积误差，
	在更长的时间尺度内都是准确的。
	但是加速度传感器测角度的缺点是加速度传感器实际上是用MEMS技术检测惯性力造成的微小形变，
	而惯性力与重力本质是一样的,所以加速度计就不会区分重力加速度与外力加速度，当系统在三维空间做变速运动时，它的输出就不正确了。

	陀螺仪输出角速度是瞬时量，角速度在姿态平衡上不能直接使用， 
	需要角速度与时间积分计算角度，得到的角度变化量与初始角度相加，
	就得到目标角度，其中积分时间Dt越小输出的角度越精确。
	但陀螺仪的原理决定了它的测量基准是自身，并没有系统外的绝对参照物，
	加上Dt是不可能无限小的，所以积分的累积误差会随着时间的流逝迅速增加，最终导致输出角度与实际不符，所以陀螺仪只能工作在相对较短的时间尺度内  。
	所以在没有其它参照物的基础上，要得到较为真实的姿态角，
	就要利用加权算法扬长避短，结合两者的优点，摈弃其各自缺点,设计算法在短时间尺度内增加陀螺仪的权值，
	在更长时间尺度内增加加速度权值，这样系统输出角度就接近真实值了。

*/

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
