// Copyright @2018 Pony AI Inc. All rights reserved.

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework4/display/main_window.h"
#include "homework4/map/map_lib.h"
#include <bits/stdc++.h>

const int test_case = 5;

double sqr(double x) {return x * x;}
double distance(interface::geometry::Point2D a, interface::geometry::Point2D b) {
  return sqrt(sqr(a.x() - b.x()) + sqr(a.y() - b.y()));
}
double distance(interface::geometry::Point2D a, interface::geometry::Point3D b) {
  return sqrt(sqr(a.x() - b.x()) + sqr(a.y() - b.y()));
}
double distance(interface::geometry::Point3D a, interface::geometry::Point3D b) {
  return sqrt(sqr(a.x() - b.x()) + sqr(a.y() - b.y()) + sqr(a.z() - b.z()));
}

int main() {
  interface::map::Map map;
  interface::route::Route route;

  file::ReadTextFileToProto("/home/vectorzhou/AutoDriving/homework4/prob2/processed_map_proto.txt", &map);
  file::ReadTextFileToProto("/home/vectorzhou/AutoDriving/homework4/data/routes/route_request_" + std::to_string(test_case) + ".txt", &route);

  int num_lane = map.lane_size();

  double x_st = route.start_point().x(), y_st = route.start_point().y(), dst_st = 999999999;
  double x_ed = route.end_point().x(), y_ed = route.end_point().y(), dst_ed = 999999999;

  // find the nearest points on the lane
  std::map <std::string, int> idmapping;
  for (int i = 0; i < num_lane; ++i) {
    interface::map::Lane li = map.lane(i);
    idmapping[li.id().id()] = i;
    int sn = li.central_line().point_size();
    for (int j = 0; j < sn; ++j) {
      double x = li.central_line().point(j).x(), y = li.central_line().point(j).y();
      double d_st = (x - x_st) * (x - x_st) + (y - y_st) * (y - y_st);
      double d_ed = (x - x_ed) * (x - x_ed) + (y - y_ed) * (y - y_ed);
      if (j != sn - 1 && d_st < dst_st) dst_st = d_st;
      if (j != 0 && d_ed < dst_ed) dst_ed = d_ed;
    }
  }

  int id_st, id_ed, point_id_st, point_id_ed;
  for (int i = 0; i < num_lane; ++i) {
    interface::map::Lane li = map.lane(i);
    int sn = li.central_line().point_size();
    for (int j = 0; j < sn - 1; ++j) {
      double x = li.central_line().point(j).x(), y = li.central_line().point(j).y();
      double d_st = (x - x_st) * (x - x_st) + (y - y_st) * (y - y_st);
      double d_ed = (x - x_ed) * (x - x_ed) + (y - y_ed) * (y - y_ed);
      if (j != sn - 1 && d_st < dst_st + 1e-9) {id_st = i; point_id_st = j + 1;}
      if (j != 0 && d_ed < dst_ed + 1e-9) {id_ed = i; point_id_ed = j - 1;}
    }
  }

  // starting point and ending point are on the same lane, and they are in the correct direction
  // directly output the route
  if (id_st == id_ed && point_id_st <= point_id_ed) {
    for (int j = point_id_st; j <= point_id_ed; ++j) {
      interface::geometry::Point2D *route_pnt = route.add_route_point();
      route_pnt->set_x(map.lane(id_st).central_line().point(j).x());
      route_pnt->set_y(map.lane(id_st).central_line().point(j).y());
    }
    file::WriteProtoToTextFile(route, "/home/vectorzhou/AutoDriving/homework4/prob3/route_result_" + std::to_string(test_case) + ".txt");
    return 0;
  }

  // try to solve with shortest path
  // build graph
  int n = 2 + num_lane * 2;
  std::vector <std::pair <int, double> > edges[n];
  for (int i = 0; i < num_lane; ++i) {
    interface::map::Lane li = map.lane(i);
    int sn = li.central_line().point_size();
    double sum = 0;
    for (int j = 1; j < sn; ++j)
      sum += distance(li.central_line().point(j - 1), li.central_line().point(j));
    edges[2 * i + 2].push_back(std::make_pair(2 * i + 3, sum));
    for (auto succ : li.successor()) {
      int j = idmapping[succ.id()];
      edges[2 * i + 3].push_back(std::make_pair(2 * j + 2, 
        distance(li.central_line().point(sn - 1),
          map.lane(j).central_line().point(0))));
    }
  }

  double sum = distance(route.start_point(), map.lane(id_st).central_line().point(point_id_st));
  if (point_id_st < map.lane(id_st).central_line().point_size() - 1) {
    interface::map::Lane li = map.lane(id_st);
    int sn = li.central_line().point_size();
    for (int j = point_id_st + 1; j < sn; ++j)
      sum += distance(li.central_line().point(j - 1), li.central_line().point(j));
  }
  edges[0].push_back(std::make_pair(2 * id_st + 3, sum));

  sum = distance(route.end_point(), map.lane(id_ed).central_line().point(point_id_ed));
  if (point_id_ed > 0) {
    interface::map::Lane li = map.lane(id_ed);
    for (int j = 1; j <= point_id_st; ++j)
      sum += distance(li.central_line().point(j - 1), li.central_line().point(j));
  }
  edges[2 * id_ed + 2].push_back(std::make_pair(1, sum));

  // dijkstra
  double dist[n];
  dist[0] = 0;
  for (int i = 1; i < n; ++i) dist[i] = 999999999;
  bool vis[n];
  memset(vis, 0, sizeof(vis));
  int pre[n];

  std::priority_queue <std::pair <double, int>, std::vector <std::pair <double, int> >, std::greater <std::pair <double, int> > > Q;
  Q.push(std::make_pair(0, 0));
  while (!Q.empty()) {
    int u = Q.top().second; Q.pop();
    if (vis[u]) continue;
    vis[u] = true;
    for (auto edge : edges[u]) {
      int v = edge.first;
      double w = edge.second;
      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        pre[v] = u;
        Q.push(std::make_pair(dist[v], v));
      }
    }
  }

  std::vector <int> route_id;
  int cur = 1;
  while (cur) {
    route_id.push_back(cur);
    cur = pre[cur];
  }
  std::reverse(route_id.begin(), route_id.end());
  for (int j = point_id_st; j < map.lane(id_st).central_line().point_size(); ++j) {
    interface::geometry::Point2D *route_pnt = route.add_route_point();
    route_pnt->set_x(map.lane(id_st).central_line().point(j).x());
    route_pnt->set_y(map.lane(id_st).central_line().point(j).y());
  }
  for (auto x : route_id) {
    if (x == 2 * id_ed + 2) break;
    if (x & 1) continue;
    int i = (x - 2) / 2;
    interface::map::Lane li = map.lane(i);
    int sn = li.central_line().point_size();
    for (int j = 0; j < sn; ++j) {
      interface::geometry::Point2D *route_pnt = route.add_route_point();
      route_pnt->set_x(map.lane(i).central_line().point(j).x());
      route_pnt->set_y(map.lane(i).central_line().point(j).y());
    }
  }
  for (int j = 0; j <= point_id_ed; ++j) {
    interface::geometry::Point2D *route_pnt = route.add_route_point();
    route_pnt->set_x(map.lane(id_ed).central_line().point(j).x());
    route_pnt->set_y(map.lane(id_ed).central_line().point(j).y());
  }

  // starting point and ending point are on the same lane, but they are in the reversed direction
  // enumerate the next lane, then use shortest path, to find the minimum cycle

  file::WriteProtoToTextFile(route, "/home/vectorzhou/AutoDriving/homework4/prob3/route_result_" + std::to_string(test_case) + ".txt");
  return 0;
}
