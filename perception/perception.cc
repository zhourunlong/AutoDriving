// Copyright @2020 Pony AI Inc. All rights reserved.

#include "perception/perception.h"

#include <limits>

void Perception::FillType(interface::perception::PerceptionObstacle* obstacle) const {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double max_y = std::numeric_limits<double>::min();
  for (auto point : obstacle->polygon_point()) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }
  if (max_x - min_x > 3.0 || max_y - min_y > 3.0) {
    obstacle->set_type(interface::perception::ObjectType::CAR);
    return;
  }
  if (obstacle->speed() > 2.0 || max_x - min_x > 1.5 || max_y - min_y > 1.5) {
    obstacle->set_type(interface::perception::ObjectType::CYCLIST);
    return;
  }
  obstacle->set_type(interface::perception::ObjectType::PEDESTRIAN);
  return;
}

void Perception::RemoveUninterestingPoints(std::vector<Perception::LaserPoint>* points) const {
  std::map<std::pair<int, int>, double> ground_height;
  std::vector<Perception::LaserPoint> kept_points;
  for (const auto& point : *points) {
    if (!point.is_in_roi) {
      continue;
    }
    std::pair<int, int> index{static_cast<int>(point.x / 3), static_cast<int>(point.y / 3)};
    if (ground_height.find(index) == ground_height.end()) {
      ground_height[index] = point.z;
    } else {
      ground_height[index] = std::min(ground_height[index], point.z);
    }
  }
  for (const auto& point : *points) {
    if (!point.is_in_roi) {
      continue;
    }
    std::pair<int, int> index{static_cast<int>(point.x / 3), static_cast<int>(point.y / 3)};
    const double height = point.z - ground_height[index];
    if (height >= 0.2 && height < 2.0) {
      kept_points.push_back(point);
    }
  }
  *points = kept_points;
}

void Perception::Floodfill(const std::vector<Perception::LaserPoint>& points,
                           std::vector<Perception::Polygon2d>* polygons) const {
  std::set<std::pair<int, int>> occupied;
  for (auto point : points) {
    std::pair<int, int> index{static_cast<int>(point.x), static_cast<int>(point.y)};
    occupied.insert(index);
  }
  while (occupied.size() > 0) {
    auto index = *occupied.begin();
    std::vector<std::pair<int, int>> cells = {index};
    int p = 0;
    while (p < cells.size()) {
      for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
          std::pair<int, int> neighbor{index.first + dx, index.second + dy};
          if (occupied.count(neighbor)) {
            occupied.erase(neighbor);
            cells.push_back(neighbor);
          }
        }
      }
      p++;
    }
    const double min_x = 0.5 + std::min_element(cells.begin(), cells.end())->first;
    const double max_x = 0.5 + std::max_element(cells.begin(), cells.end())->first;
    auto compare_second = [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
      return a < b;
    };
    const double min_y = 0.5 + std::min_element(cells.begin(), cells.end(), compare_second)->second;
    const double max_y = 0.5 + std::max_element(cells.begin(), cells.end(), compare_second)->second;
    polygons->push_back({math::Vec2d(min_x, min_y), math::Vec2d(min_x, max_y),
                         math::Vec2d(max_x, max_y), math::Vec2d(max_x, min_y)});
  }
}

void Perception::FillPrism(const Perception::Polygon2d& polygon,
                           double height,
                           interface::perception::PerceptionObstacle* obstacle) const {
  obstacle->set_height(height);
  for (const auto& point : polygon) {
    auto* polygon_point = obstacle->add_polygon_point();
    polygon_point->set_x(point.x);
    polygon_point->set_y(point.y);
    polygon_point->set_z(z_);
  }
}

void Perception::FillVelocity(const math::Vec2d& previous_center,
                              const math::Vec2d& center,
                              interface::perception::PerceptionObstacle* obstacle) const {
  const math::Vec2d& offset = center - previous_center;
  obstacle->set_speed(offset.Length() * 10);
  obstacle->set_heading(std::atan2(offset.y, offset.x));
}

interface::perception::PerceptionObstacles Perception::OneIteration(const PointCloud& pointcloud) {
  interface::perception::PerceptionObstacles perception_result;
  std::vector<LaserPoint> points;
  z_ = pointcloud.translation.z();
  for (int i = 0; i < pointcloud.points.size(); ++i) {
    const auto& point = pointcloud.points[i];
    auto world_pos = pointcloud.rotation * point + pointcloud.translation;
    points.push_back({.x = world_pos.x(), .y = world_pos.y(), .z = world_pos.z(),
                      .is_in_roi = pointcloud.is_in_roi[i]});
  }
  RemoveUninterestingPoints(&points);
  std::vector<Polygon2d> polygons;
  Floodfill(points, &polygons);
  std::vector<math::Vec2d> centers;
  for (const auto& polygon : polygons) {
    auto* obstacle = perception_result.add_obstacle();
    const math::Vec2d center =
        std::accumulate(polygon.begin(), polygon.end(), math::Vec2d()) /
        static_cast<double>(polygon.size());
    centers.push_back(center);
    math::Vec2d previous_center = previous_centers_.size() > 0 ?
        *std::min_element(previous_centers_.begin(), previous_centers_.end(),
            [&center] (const math::Vec2d& p_center1, const math::Vec2d& p_center2){
              return (p_center1 - center).Length() < (p_center2 - center).Length();
            }) : center;
    if ((previous_center - center).Length() > 4.0) {
      previous_center = center;
    }
    FillPrism(polygon, 1.5, obstacle);
    FillVelocity(previous_center, center, obstacle);
    FillType(obstacle);
  }
  previous_centers_ = centers;
  return perception_result;
}