// Copyright @2020 Pony AI Inc. All rights reserved.

#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common/proto/perception.pb.h"
#include "common/utils/common/defines.h"
#include "common/utils/common/optional.h"
#include "common/utils/math/vec2d.h"
#include "homework2/pointcloud.h"

class Perception {
 public:
  Perception() = default;

  interface::perception::PerceptionObstacles OneIteration(const PointCloud& pointcloud);

 private:
  using Polygon2d = std::vector<math::Vec2d>;
  struct LaserPoint{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool is_in_roi = false;
  };
  void RemoveUninterestingPoints(std::vector<LaserPoint>* points) const;
  void Floodfill(const std::vector<LaserPoint>& points, std::vector<Polygon2d>* polygons) const;
  void FillPrism(const Polygon2d& polygon, double height,
      interface::perception::PerceptionObstacle* obstacle) const;
  void FillVelocity(const math::Vec2d& previous_center,const math::Vec2d& center,
      interface::perception::PerceptionObstacle* obstacle) const;
  void FillType(interface::perception::PerceptionObstacle* obstacle) const;

  double z_ = 0.0;

  std::vector<math::Vec2d> previous_centers_;
  // You can feel free to add some private variables to record tracking information. You can match
  // the obstacles you gotten in the current iteration with tracks to get their velocity.
  DISALLOW_COPY_MOVE_AND_ASSIGN(Perception);
};

