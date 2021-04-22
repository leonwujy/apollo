/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/base/lane_struct.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/common/data_provider.h"

namespace apollo {
namespace perception {
namespace camera {

class BaseCalibrationService;

//相机图像
struct CameraFrame {
  // timestamp
  double timestamp = 0.0; //时间戳
  // frame sequence id
  int frame_id = 0;       //帧号
  // data provider
  DataProvider *data_provider = nullptr; //数据提供者
  // calibration service
  BaseCalibrationService *calibration_service = nullptr;  //标定服务
  // hdmap struct
  base::HdmapStructPtr hdmap_struct = nullptr; //高清地图
  // tracker proposed objects
  std::vector<base::ObjectPtr> proposed_objects; //提议的目标
  // segmented objects
  std::vector<base::ObjectPtr> detected_objects; //检测到的目标
  // tracked objects
  std::vector<base::ObjectPtr> tracked_objects;  //跟踪的目标
  // feature of all detected object ( num x dim)
  // detect lane mark info
  std::vector<base::LaneLine> lane_objects;      //车道线目标
  std::vector<float> pred_vpt;                   //预测的vpt？
  std::shared_ptr<base::Blob<float>> track_feature_blob = nullptr;
  std::shared_ptr<base::Blob<float>> lane_detected_blob = nullptr;
  // detected traffic lights
  // 检测到的交通信号灯
  std::vector<base::TrafficLightPtr> traffic_lights;
  // camera intrinsics
  // 相机内参
  Eigen::Matrix3f camera_k_matrix = Eigen::Matrix3f::Identity();
  // narrow to obstacle projected_matrix
  // 投射矩阵 3x3
  Eigen::Matrix3d project_matrix = Eigen::Matrix3d::Identity();
  // camera to world pose
  // 从相机坐标系到世界坐标系的姿态投影
  Eigen::Affine3d camera2world_pose = Eigen::Affine3d::Identity();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // struct CameraFrame

}  // namespace camera
}  // namespace perception
}  // namespace apollo
