/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once
#include <memory>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>
#include "Eigen/Core"

#include "modules/perception/base/object_supplement.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/base/vehicle_struct.h"
// #include "modules/prediction/proto/feature.pb.h"

namespace apollo {
namespace perception {
namespace base {

//目标信息类
struct alignas(16) Object {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Object();
  std::string ToString() const;
  void Reset();

  // @brief object id per frame, required
  // 目标编号
  int id = -1;

  // @brief convex hull of the object, required
  // 目标物体的凸包
  PointCloud<PointD> polygon;

  // oriented boundingbox information
  // @brief main direction of the object, required
  // 目标物体的朝向，3D矢量
  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
  /*@brief the yaw angle, theta = 0.0 <=> direction(1, 0, 0),
    currently roll and pitch are not considered,
    make sure direction and theta are consistent, required
  */
  //目标的偏航角 yaw（即，车辆在水平地面上的车头转向朝向），俯仰角pitch和翻滚角roll不考虑
  float theta = 0.0f;
  // @brief theta variance, required
  //角度方差
  float theta_variance = 0.0f;
  // @brief center of the boundingbox (cx, cy, cz), required
  //物体的3D中心点
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the center uncertainty, required
  // 中心不确定度的协方差矩阵
  Eigen::Matrix3f center_uncertainty;
  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  //物体的三维尺寸，以物体的朝向为长度方向
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  // @brief size variance, required
  //三个尺寸上的方差
  Eigen::Vector3f size_variance = Eigen::Vector3f(0, 0, 0);
  // @brief anchor point, required
  // 3D锚点？是否是滤波后的中心点？
  Eigen::Vector3d anchor_point = Eigen::Vector3d(0, 0, 0);

  // @brief object type, required
  // 物体类型
  ObjectType type = ObjectType::UNKNOWN;
  // @brief probability for each type, required
  // 物体属于各类别的概率
  std::vector<float> type_probs;

  // @brief object sub-type, optional
  // 物体子类别
  ObjectSubType sub_type = ObjectSubType::UNKNOWN;
  // @brief probability for each sub-type, optional
  // 子类别概率
  std::vector<float> sub_type_probs;

  // @brief existence confidence, required
  // 置信度
  float confidence = 1.0f;

  // tracking information（跟踪信息）
  // @brief track id, required
  // 跟踪的ID号
  int track_id = -1;
  // @brief velocity of the object, required
  // 物体速度矢量
  Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  // 速度矢量不确定度的协方差矩阵
  Eigen::Matrix3f velocity_uncertainty;
  // @brief if the velocity estimation is converged, true by default
  // 速度估计是否收敛
  bool velocity_converged = true;
  // @brief velocity confidence, required
  // 速度置信度
  float velocity_confidence = 1.0f;
  // @brief acceleration of the object, required
  // 物体的加速度矢量
  Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the acceleration uncertainty, required
  // 加速度矢量不确定度的协方差矩阵
  Eigen::Matrix3f acceleration_uncertainty;

  // @brief age of the tracked object, required
  // 跟踪时间
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  // 上次检测到该物体的时间戳
  double latest_tracked_time = 0.0;

  // @brief motion state of the tracked object, required
  // 被跟踪物体的运动状态
  MotionState motion_state = MotionState::UNKNOWN;
  // // Tailgating (trajectory of objects)
  // 物体的跟踪轨迹
  std::array<Eigen::Vector3d, 100> drops;
  std::size_t drop_num = 0;
  // // CIPV
  bool b_cipv = false;
  // @brief brake light, left-turn light and right-turn light score, optional
  // 车灯信息
  CarLight car_light;
  // @brief sensor-specific object supplements, optional
  // 特定传感器提供的补充信息
  LidarObjectSupplement lidar_supplement;
  RadarObjectSupplement radar_supplement;
  CameraObjectSupplement camera_supplement;
  FusionObjectSupplement fusion_supplement;

  // @debug feature to be used for semantic mapping
//  std::shared_ptr<apollo::prediction::Feature> feature;
};

using ObjectPtr = std::shared_ptr<Object>;
using ObjectConstPtr = std::shared_ptr<const Object>;

}  // namespace base
}  // namespace perception
}  // namespace apollo
