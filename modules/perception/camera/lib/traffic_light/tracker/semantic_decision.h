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

#include <string>
#include <vector>

#include "modules/perception/camera/lib/interface/base_traffic_light_tracker.h"
#include "modules/perception/camera/lib/traffic_light/tracker/proto/semantic.pb.h"

namespace apollo {
namespace perception {
namespace camera {

//滞回窗口
struct HystereticWindow {
  int hysteretic_count = 0;
  base::TLColor hysteretic_color = base::TLColor::TL_UNKNOWN_COLOR;
};

//语义表
struct SemanticTable {
  double time_stamp = 0.0;
  double last_bright_time_stamp = 0.0;   //最近亮灯的时刻
  double last_dark_time_stamp = 0.0;     //最近灭灯的时刻
  bool blink = false;
  std::string semantic;       //语义
  std::vector<int> light_ids; //交通灯序号
  base::TLColor color;        //交通灯颜色
  HystereticWindow hystertic_window;  //回滞窗大小
};

//语义修正器
class SemanticReviser : public BaseTrafficLightTracker {
 public:
  SemanticReviser() {}
  ~SemanticReviser() {}

  bool Init(const TrafficLightTrackerInitOptions &options =
                TrafficLightTrackerInitOptions()) override;

  bool Track(const TrafficLightTrackerOptions &options,
             CameraFrame *frame) override;
  //通过语义修正
  base::TLColor ReviseBySemantic(SemanticTable semantic_table,
                                 std::vector<base::TrafficLightPtr> *lights);
  //通过时间序列修正
  void ReviseByTimeSeries(double time_stamp, SemanticTable semantic_table,
                          std::vector<base::TrafficLightPtr> *lights);
  //更新历史和交通灯
  void UpdateHistoryAndLights(const SemanticTable &cur,
                              std::vector<base::TrafficLightPtr> *lights,
                              std::vector<SemanticTable>::iterator *history);
  //修正交通灯
  void ReviseLights(std::vector<base::TrafficLightPtr> *lights,
                    const std::vector<int> &light_ids, base::TLColor dst_color);

  std::string Name() const override;

  explicit SemanticReviser(const BaseTrafficLightTracker &) = delete;
  SemanticReviser &operator=(const BaseTrafficLightTracker &) = delete;

 private:
  traffic_light::tracker::SemanticReviseParam semantic_param_;
  float revise_time_s_ = 1.5f;           //修订时间(秒)
  float blink_threshold_s_ = 0.4f;       //闪烁阈值(秒)
  float non_blink_threshold_s_ = 0.8f;   //无闪烁阈值(秒)
  int hysteretic_threshold_ = 1;         //回滞阈值
  std::vector<SemanticTable> history_semantic_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
