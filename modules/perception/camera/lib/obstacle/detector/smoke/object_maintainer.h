/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/base/object.h"
#include "modules/perception/base/object_pool_types.h"

namespace apollo {
namespace perception {
namespace camera {

//smoke检测模型主类
class SmokeObjectMaintainer {
 public:
  SmokeObjectMaintainer() {}
  ~SmokeObjectMaintainer() {}
  bool Add(int idx, base::ObjectPtr obj); //添加目标及其序号
 protected:
  std::map<int, base::ObjectPtr> assigned_index_; //序号和对象的映射表
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
