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

namespace apollo {
namespace perception {
namespace base {

// 目标车灯的置信度信息（先检测到灯，再判断灯的状态？）
struct CarLight {
  float brake_visible = 0.0f;          //刹车灯可见度
  float brake_switch_on = 0.0f;        //刹车灯开启度
  float left_turn_visible = 0.0f;      //左转灯可见度
  float left_turn_switch_on = 0.0f;    //左转灯开启度
  float right_turn_visible = 0.0f;     //右转灯可见度
  float right_turn_switch_on = 0.0f;   //右转灯开启度

  //重置所有信息
  void Reset() {
    brake_visible = 0;
    brake_switch_on = 0;
    left_turn_visible = 0;
    left_turn_switch_on = 0;
    right_turn_visible = 0;
    right_turn_switch_on = 0;
  }
};

}  // namespace base
}  // namespace perception
}  // namespace apollo
