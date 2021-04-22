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

#include <map>
#include <string>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

//标定服务初始化设置
struct CalibrationServiceInitOptions : public BaseInitOptions {
  int image_width = 0;
  int image_height = 0;
  double timestamp = 0;
  std::string calibrator_working_sensor_name = "";  //标定工作传感器名称
  std::string calibrator_method = "";               //标定方式名称
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map; //名字-内参数据对照表
};

//标定服务设置
struct CalibrationServiceOptions {};

//标定服务
class BaseCalibrationService {
 public:
  BaseCalibrationService() = default;

  virtual ~BaseCalibrationService() = default;

  virtual bool Init(const CalibrationServiceInitOptions &options =
                        CalibrationServiceInitOptions()) = 0;

  virtual bool BuildIndex() = 0;

  // @brief query camera to world pose with refinement if any
  // 如果有的话，查询到相机相对世界的姿态
  virtual bool QueryCameraToWorldPose(Eigen::Matrix4d *pose) const {
    return false;
  }

  // @brief query depth on ground plane given pixel coordinate
  // 查询给定像素坐标在地平面上的深度
  virtual bool QueryDepthOnGroundPlane(int x, int y, double *depth) const {
    return false;
  }

  // @brief query 3d point on ground plane given pixel coordinate
  // 查询给定像素坐标在地平线上的3D点坐标
  virtual bool QueryPoint3dOnGroundPlane(int x, int y,
                                         Eigen::Vector3d *point3d) const {
    return false;
  }

  // @brief query ground plane in camera frame, parameterized as
  // [n^T, d] with n^T*x+d=0
  // 查询相机坐标系中的地平面，返回结构以[n^T, d]呈现 n^T*x+d=0 
  virtual bool QueryGroundPlaneInCameraFrame(
      Eigen::Vector4d *plane_param) const {
    return false;
  }

  // @brief query camera to ground height and pitch angle
  //查询相机相对地面的高度，以及相机俯仰角
  virtual bool QueryCameraToGroundHeightAndPitchAngle(float *height,
                                                      float *pitch) const {
    return false;
  }

  //查询相机相对地面的高度
  virtual float QueryCameraToGroundHeight() const { return 0.f; }

  //查询相机俯仰角
  virtual float QueryPitchAngle() const { return 0.f; }

  // @brief using calibrator to update pitch angle
  // 用标定信息来更新俯仰角
  virtual void Update(CameraFrame *frame) {
    // do nothing
  }

  // @brief set camera height, pitch and project matrix
  virtual void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_master_sensor) {
    // do nothing
  }

  virtual std::string Name() const = 0;

  BaseCalibrationService(const BaseCalibrationService &) = delete;
  BaseCalibrationService &operator=(const BaseCalibrationService &) = delete;
};  // class BaseCalibrationService

PERCEPTION_REGISTER_REGISTERER(BaseCalibrationService);
#define REGISTER_CALIBRATION_SERVICE(name) \
  PERCEPTION_REGISTER_CLASS(BaseCalibrationService, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
