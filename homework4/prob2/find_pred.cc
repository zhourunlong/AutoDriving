// Copyright @2018 Pony AI Inc. All rights reserved.

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework4/display/main_window.h"
#include "homework4/map/map_lib.h"
#include <bits/stdc++.h>

int main() {
  interface::map::Map map;
  interface::map::Map ret;

  file::ReadTextFileToProto("/home/vectorzhou/AutoDriving/homework4/map/grid2/map_proto.txt", &map);
  
  int num_lane = map.lane_size();
  
  std::vector <int> pred[num_lane], succ[num_lane];
  for (int i = 0; i < num_lane; ++i) {
    interface::map::Lane li = map.lane(i);
    int sn = li.central_line().point_size();
    interface::geometry::Point3D end_i = li.central_line().point(sn - 1);
    double xi = end_i.x(), yi = end_i.y(), zi = end_i.z();

    for (int j = 0; j < num_lane; ++j) {
      interface::geometry::Point3D end_j = map.lane(j).central_line().point(0);
      double xj = end_j.x(), yj = end_j.y(), zj = end_j.z();

      if ((xi - xj) * (xi - xj) + (yi - yj) * (yi - yj) + (zi - zj) * (zi - zj) < 1) {
        succ[i].push_back(j);
        pred[j].push_back(i);
      }
    }
  }

  int num_light = map.traffic_light_size();
  for (int i = 0; i < num_light; ++i) {
    interface::map::TrafficLight *tl = ret.add_traffic_light();
    *tl = map.traffic_light(i);
  }

  for (int i = 0; i < num_lane; ++i) {
    interface::map::Lane *li = ret.add_lane();
    *li = map.lane(i);
    
    for (auto j : succ[i]) {
      interface::map::Id *new_id = li->add_successor();
      new_id->set_id(map.lane(j).id().id());
    }

    for (auto j : pred[i]) {
      interface::map::Id *new_id = li->add_predecessor();
      new_id->set_id(map.lane(j).id().id());
    }
  }

  file::WriteProtoToTextFile(ret, "/home/vectorzhou/AutoDriving/homework4/prob2/processed_map_proto.txt");
  return 0;
}
