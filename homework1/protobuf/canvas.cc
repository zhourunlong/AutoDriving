// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework1/protobuf/canvas.h"

#include <bits/stdc++.h>
#include <glog/logging.h>

template<typename T> T sqr(T x) {return x * x;}

namespace homework1 {

using homework1::geometry::Point3D;

void Canvas::Draw() const {
  for (const auto& p : polygon_.point()) {
    std::cout << "Point:" << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  }
}

void Canvas::AddPoint(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint(point);
}

void Canvas::AddPoint(const Point3D& p) {
  auto* point = polygon_.add_point();
  point->CopyFrom(p);
}

const Point3D& Canvas::GetPoint(int index) const {
  return polygon_.point(index);
}

void Canvas::ParseFromString(const std::string& serialzation) {
  polygon_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString() const {
  std::string serialzation;
  CHECK(polygon_.SerializeToString(&serialzation)) << "Canvas serialization failed.";
  return serialzation;
}

double Canvas::ComputeLength() const {
  double len = 0;
  bool f = false;
  Point3D q;
  for (const auto& p: polygon_.point()) {
    if (f) len += ::sqrt(sqr(p.x() - q.x()) + sqr(p.y() - q.y()) + sqr(p.z() - q.z()));
    f = true;
    q = p;
  }
  return len;
}

}  // namespace homework1
