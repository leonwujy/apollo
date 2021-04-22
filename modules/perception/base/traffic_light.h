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

#include "modules/perception/base/box.h"

namespace apollo {
namespace perception {
namespace base {

// 交通灯颜色
enum class TLColor {
  TL_UNKNOWN_COLOR = 0,
  TL_RED = 1,
  TL_YELLOW = 2,
  TL_GREEN = 3,
  TL_BLACK = 4,
  TL_TOTAL_COLOR_NUM = 5
};

//交通灯类型
enum class TLDetectionClass {
  TL_UNKNOWN_CLASS = -1,    
  TL_VERTICAL_CLASS = 0,    //垂直交通灯
  TL_QUADRATE_CLASS = 1,    //立方体交通灯
  TL_HORIZONTAL_CLASS = 2   //水平交通灯
};

// @brief Light Region in the Image
// 图像中的光亮区域
struct LightRegion {
  // roi is marked by map & projection, it may be too large or not accuracy.
  // ROI是用地图和投影来表示的，它可能太大或精度不高
  Rect<int> projection_roi; //投影得到的ROI
  Rect<int> crop_roi;       //裁剪ROI？
  bool outside_image = false;

  std::vector<Rect<int>> debug_roi;
  std::vector<float> debug_roi_detect_scores;

  Rect<int> detection_roi;
  bool is_detected = false;
  bool is_selected = false;
  TLDetectionClass detect_class_id = TLDetectionClass::TL_UNKNOWN_CLASS;

  // 3d polygon
  // 3D包络多边形顶点
  std::vector<base::PointXYZID> points;

  // output score by detection
  // 输出的检测分数
  float detect_score = 0.0f;
};

// @brief Light Status
// 灯的状态
struct LightStatus {
  // Traffic light color status.
  // 交通灯颜色
  TLColor color = TLColor::TL_UNKNOWN_COLOR;
  // How confidence about the detected results, between 0 and 1.
  // 交通灯置信度
  double confidence = 0.0;
  // Duration of the traffic light since detected.
  // 交通灯被检测到后的持续时间
  double tracking_time = 0.0;
  // blink status
  // 是否闪烁
  bool blink = false;
};

// @brief A Traffic Light.
// 交通灯
struct TrafficLight {
  TrafficLight() = default;

  std::string id;
  int semantic = 0;
  //交通灯的区域和状态
  LightRegion region;  // Light region.
  LightStatus status;  // Light Status.
};

typedef std::shared_ptr<TrafficLight> TrafficLightPtr;
typedef std::vector<TrafficLightPtr> TrafficLightPtrs;

}  // namespace base
}  // namespace perception
}  // namespace apollo
