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
#include <vector>

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace base {

//道路边界信息
struct alignas(16) RoadBoundary {
  PointCloud<PointD> left_boundary;  //左边界的3D坐标点集合
  PointCloud<PointD> right_boundary; //右边界的3D坐标点集合
};

//车道边界
struct alignas(16) LaneBoundary {
  PointCloud<PointD> left_boundary;  //左边界的3D坐标点集合
  PointCloud<PointD> right_boundary; //右边界的3D坐标点集合
};

//16字节内存对齐的高清地图结构体
struct alignas(16) HdmapStruct {
  apollo::common::EigenVector<RoadBoundary> road_boundary;       //道路边界信息
  apollo::common::EigenVector<PointCloud<PointD>> road_polygons; //道路多边形点集
  apollo::common::EigenVector<PointCloud<PointD>> hole_polygons; //孔洞型多边形点集
  apollo::common::EigenVector<PointCloud<PointD>> junction_polygons; //连接多边形点集
};

using HdmapStructPtr = std::shared_ptr<HdmapStruct>;
using HdmapStructConstPtr = std::shared_ptr<const HdmapStruct>;

}  // namespace base
}  // namespace perception
}  // namespace apollo
