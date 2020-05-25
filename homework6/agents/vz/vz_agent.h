// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/proto/route.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"
#include "common/utils/file/file.h"
#include "homework6/map/map_lib.h"
#include <bits/stdc++.h>

namespace vz {

const double eps = 1e-2;
const double alpha = 5;
const double beta = -0.0005;
const double throttle_table[81] = {0,0.00740,0.01396,0.01975,0.02484,0.02930,0.03317,0.03652,0.03938,0.04181,0.04384,0.04551,0.04686,0.04792,0.04871,0.04927,0.10940,0.10353,0.09291,0.11906,0.13142,0.13174,0.14947,0.17174,0.19190,0.21569,0.26078,0.30588,0.35098,0.39608,0.44118,0.48627,0.53137,0.57647,0.63823,0.70000,0.79882,0.89765,0.99647,1.09529,1.19412,1.32588,1.45765,1.58941,1.72118,1.85294,1.92706,2.00118,2.07529,2.14941,2.22353,2.29765,2.37176,2.44588,2.52000,2.59412,2.66823,2.74235,2.81647,2.89059,2.96470,3.01412,3.06353,3.11294,3.16235,3.21176,3.26117,3.31059,3.36000,3.40941,3.45882,3.53706,3.61529,3.69353,3.77176,3.85000,3.92823,4.00647,4.08470,4.16294,4.24117};
const double brake_table[41] = {0,0.02417,0.04835,0.07252,0.09670,0.12087,0.14504,0.16922,0.19339,0.21757,0.24174,0.26591,0.29009,0.31426,0.33844,0.35556,0.36209,0.36863,0.37516,0.38170,0.38824,0.56471,0.74118,1.16471,1.65882,2.15294,2.75294,3.35294,3.96078,4.56863,5.17647,5.78823,6.40000,7.01176,7.62353,8.23529,8.70588,9.17647,9.64705,10.11764,10.58823};
const int num_predict = 10;
const double pi = acos(-1);
const double Alpha = -1;
const double Beta = 0;
const double target_velocity = 5;

struct PID {
  double Kp, Ki, Kd;
  double et_last, et_integral;
  bool is_first_iter;
  PID(double KP, double KI, double KD): Kp(KP), Ki(KI), Kd(KD) {
    is_first_iter = true;
    et_integral = 0;
  }
  double calc(double et) {
    et_integral += et * 0.01;
    if (is_first_iter) {
      et_last = et;
      is_first_iter = false;
    }
    double ut = et + Ki * et_integral + Kd * (et - et_last) * 100;

    //std::cerr << et << " " << et_last << " " << et_integral << " " << ut << "\n";
    et_last = et;
    return ut;
  }
};

class vzVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit vzVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& /* agent_status */) {
    // Nothing to initialize
  }

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {

    if (iters == 0)
      FindRoute(agent_status);

    while (route_pnt.size() > 1 && 
      (VeryClose(route_pnt[0], route_pnt[1]) || 
      CalcDistance(agent_status.vehicle_status().position(), Point2Dto3D(route_pnt[0]))
      > CalcDistance(agent_status.vehicle_status().position(), Point2Dto3D(route_pnt[1]))))
      route_pnt.pop_front();

    double tar_a, tar_v;
    double acc = CalcVelocity(agent_status.vehicle_status().acceleration_vcs());
    double vel = CalcVelocity(agent_status.vehicle_status().velocity());
    double ds = distance(agent_status.vehicle_status().position(), route_pnt[0]);
    for (int i = 1; i < route_pnt.size(); ++i)
      ds += distance(route_pnt[i - 1], route_pnt[i]);

    //std::cout << iters * 0.01 << " " << vel << "\n";
    //std::cerr << iters * 0.01 << " " << vel << "\n";

    if (ds < target_velocity) {
      tar_a = -10;
      tar_v = 0;
    } else if (vel > target_velocity + eps) {
      tar_a = -4;
      tar_v = target_velocity;
    } else if (vel < target_velocity - eps) {
      tar_a = 4;
      tar_v = target_velocity;
    } else {
      tar_a = 0;
      tar_v = target_velocity;
    }

    double et = acc - tar_a + alpha * (vel - tar_v) + beta * ds;
    double ut = longitudinal.calc(et);
    
    interface::control::ControlCommand command;
    if (ut > 0)
      command.set_brake_ratio(CalcBrakeFromUt(ut));
    if (ut < 0)
      command.set_throttle_ratio(CalcThrottleFromUt(ut));
    /*
    fprintf(stderr, "(%lf, %lf)", agent_status.vehicle_status().position().x(), agent_status.vehicle_status().position().y());
    for (int i = 0; i < route_pnt.size() && i < num_predict; ++i)
      fprintf(stderr, "(%lf, %lf)", route_pnt[i].x(), route_pnt[i].y());
    */

    if (vel > 1 && route_pnt.size() > num_predict) {
      interface::geometry::Point2D Q = route_pnt[num_predict - 1];
      double angle = CalcAngle(agent_status.vehicle_status().velocity(),
        minus(Point2DtoVector3d(Q), agent_status.vehicle_status().position()));
      double L = CalcDistance(agent_status.vehicle_status().position(), Point2Dto3D(Q));
      double kappa = 2 * sin(angle) / L;
      //double curv =  / vel;
      double Et = kappa - Alpha * angle + Beta * L;
      double Ut = lateral.calc(Et);
      command.set_steering_angle(CalcSteerFromUt(Ut));
      //std::cerr << Et << " " << Ut << "\n";
      //std::cerr << angle << " " << L << " " << kappa << " " << Et << "\n";
    } else
      lateral.calc(0);

    ++iters;
    return command;
  }

 private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    return std::sqrt(sqr_sum);
  }
  
  double CalcBrakeFromUt(const double& ut) {
    double u = ut * 0.25;
    int p = std::lower_bound(brake_table, brake_table + 41, u) - brake_table;
    if (p == 41) return 0.4;
    if (p == 0) return 0;
    if (2 * u  > brake_table[p] + brake_table[p - 1]) return p * 0.01;
    return (p - 1) * 0.01;
  }

  double CalcThrottleFromUt(const double& ut) {
    double u = -ut * 0.15;
    int p = std::lower_bound(throttle_table, throttle_table + 81, u) - throttle_table;
    if (p == 81) return 0.8;
    if (p == 0) return 0;
    if (2 * u  > throttle_table[p] + throttle_table[p - 1]) return p * 0.01;
    return (p - 1) * 0.01;
  }

  double CalcSteerFromUt(const double& ut) {
    return CalcSteerFromCurv(ut * 0.5);
  }

  double CalcAngle(const interface::geometry::Vector3d& a,
    const interface::geometry::Vector3d& b) {
    
    double area = CalcVelocity(a) * CalcVelocity(b);

    double cross = a.x() * b.y() - a.y() * b.x();
    double Sin = cross / area;
    double ang1 = std::asin(Sin);

    double dot = a.x() * b.x() + a.y() * b.y();
    double Cos = dot / area;
    double ang2 = std::acos(Cos);

    if (ang1 > 0)
      if (ang2 < pi * 0.5) return ang1;
      else return ang2;
    else
      if (ang2 < pi * 0.5) return ang1;
      else return -ang2;
  }

  interface::geometry::Point2D Point3Dto2D(const interface::geometry::Point3D& p) {
    interface::geometry::Point2D q;
    q.set_x(p.x());
    q.set_y(p.y());
    return q;
  }

  interface::geometry::Point3D Point2Dto3D(const interface::geometry::Point2D& p) {
    interface::geometry::Point3D q;
    q.set_x(p.x());
    q.set_y(p.y());
    return q;
  }
  
  interface::geometry::Vector3d minus(const interface::geometry::Vector3d& p,
    const interface::geometry::Vector3d& q) {
    interface::geometry::Vector3d r;
    r.set_x(p.x() - q.x());
    r.set_y(p.y() - q.y());
    r.set_z(p.z() - q.z());
    return r;
  }

  interface::geometry::Vector3d Point2DtoVector3d(const interface::geometry::Point2D& p) {
    interface::geometry::Vector3d q;
    q.set_x(p.x());
    q.set_y(p.y());
    return q;
  }

  double distance(interface::geometry::Vector3d a, interface::geometry::Point2D b) {
    return std::sqrt(math::Sqr(a.x() - b.x()) + math::Sqr(a.y() - b.y()));
  }

  double distance(interface::geometry::Vector3d a, interface::geometry::Point3D b) {
    return std::sqrt(math::Sqr(a.x() - b.x()) + math::Sqr(a.y() - b.y()));
  }

  double distance(interface::geometry::Point2D a, interface::geometry::Point2D b) {
    return std::sqrt(math::Sqr(a.x() - b.x()) + math::Sqr(a.y() - b.y()));
  }

  double distance(interface::geometry::Point2D a, interface::geometry::Point3D b) {
    return std::sqrt(math::Sqr(a.x() - b.x()) + math::Sqr(a.y() - b.y()));
  }

  double distance(interface::geometry::Point3D a, interface::geometry::Point2D b) {
    return std::sqrt(math::Sqr(a.x() - b.x()) + math::Sqr(a.y() - b.y()));
  }

  double distance(interface::geometry::Point3D a, interface::geometry::Point3D b) {
    return std::sqrt(math::Sqr(a.x() - b.x()) + math::Sqr(a.y() - b.y()) + math::Sqr(a.z() - b.z()));
  }

  bool VeryClose(const interface::geometry::Point2D& p,
    const interface::geometry::Point2D& q) {
    return distance(p, q) < 1e-3;
  }

  double CalcSteerFromCurv(double kappa) {
    double tmp = 0.0074 * kappa - 1;
    double sqrt_delta = std::sqrt(tmp * tmp + 10.75);
    double x1, x2;
    if (tmp < 1e-9) {
      x1 = (tmp - sqrt_delta) / (0.12 * kappa);
      x2 = 89.588 / (kappa * (sqrt_delta - tmp));
    } else {
      x1 = (tmp - sqrt_delta) / (0.12 * kappa);
      x2 = -89.588 / (kappa * (sqrt_delta + tmp));
    }
    if (kappa < -1e-9) {
      if (x1 < -1e-9) return x1;
      else return x2;
    } else {
      if (x1 > 1e-9) return x1;
      else return x2;
    }
  }

  void FindRoute(const interface::agent::AgentStatus& agent_status) {
    if (iters == 0)
      file::ReadTextFileToProto("/home/vectorzhou/AutoDriving/homework6/agents/vz/processed_map_proto.txt", &map);

    int num_lane = map.lane_size();

    double x_st = agent_status.vehicle_status().position().x(), y_st = agent_status.vehicle_status().position().y(), dst_st = 999999999;
    double x_ed = agent_status.route_status().destination().x(), y_ed = agent_status.route_status().destination().y(), dst_ed = 999999999;

    // find the nearest points on the lane
    std::map <std::string, int> idmapping;
    for (int i = 0; i < num_lane; ++i) {
      interface::map::Lane li = map.lane(i);
      idmapping[li.id().id()] = i;
      int sn = li.central_line().point_size();
      for (int j = 0; j < sn; ++j) {
        double x = li.central_line().point(j).x(), y = li.central_line().point(j).y();
        double d_st = math::Sqr(x - x_st) + math::Sqr(y - y_st);
        double d_ed = math::Sqr(x - x_ed) + math::Sqr(y - y_ed);
        if (j != sn - 1 && d_st < dst_st) dst_st = d_st;
        if (j != 0 && d_ed < dst_ed) dst_ed = d_ed;
      }
    }

    int id_st, id_ed, point_id_st, point_id_ed;
    for (int i = 0; i < num_lane; ++i) {
      interface::map::Lane li = map.lane(i);
      int sn = li.central_line().point_size();
      for (int j = 0; j < sn; ++j) {
        double x = li.central_line().point(j).x(), y = li.central_line().point(j).y();
        double d_st = math::Sqr(x - x_st) + math::Sqr(y - y_st);
        double d_ed = math::Sqr(x - x_ed) + math::Sqr(y - y_ed);
        if (j != sn - 1 && d_st < dst_st + 1e-9) {id_st = i; point_id_st = j + 1;}
        if (j != 0 && d_ed < dst_ed + 1e-9) {id_ed = i; point_id_ed = j - 1;}
      }
    }

    route_pnt.clear();

    // starting point and ending point are on the same lane, and they are in the correct direction
    // directly output the route
    if (id_st == id_ed && point_id_st <= point_id_ed) {
      for (int j = point_id_st; j <= point_id_ed; ++j) 
        route_pnt.push_back(Point3Dto2D(map.lane(id_st).central_line().point(j)));
      return;
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

    double sum = distance(agent_status.vehicle_status().position(), map.lane(id_st).central_line().point(point_id_st));
    if (point_id_st < map.lane(id_st).central_line().point_size() - 1) {
      interface::map::Lane li = map.lane(id_st);
      int sn = li.central_line().point_size();
      for (int j = point_id_st + 1; j < sn; ++j)
        sum += distance(li.central_line().point(j - 1), li.central_line().point(j));
    }
    edges[0].push_back(std::make_pair(2 * id_st + 3, sum));

    sum = distance(agent_status.route_status().destination(), map.lane(id_ed).central_line().point(point_id_ed));
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
    for (int j = point_id_st; j < map.lane(id_st).central_line().point_size(); ++j)
      route_pnt.push_back(Point3Dto2D(map.lane(id_st).central_line().point(j)));
    for (auto x : route_id) {
      if (x == 2 * id_ed + 2) break;
      if (x & 1) continue;
      int i = (x - 2) / 2;
      interface::map::Lane li = map.lane(i);
      int sn = li.central_line().point_size();
      for (int j = 0; j < sn; ++j)
        route_pnt.push_back(Point3Dto2D(map.lane(i).central_line().point(j)));
    }
    for (int j = 0; j <= point_id_ed; ++j)
      route_pnt.push_back(Point3Dto2D(map.lane(id_ed).central_line().point(j)));
  }
  
  PID longitudinal = PID(1.0, 0.1, 0.3);
  PID lateral = PID(1.0, target_velocity / 10, 0.9);

  interface::map::Map map;
  std::deque <interface::geometry::Point2D> route_pnt;
  int iters = 0;

};

}
