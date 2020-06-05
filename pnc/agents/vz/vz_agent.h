// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/proto/route.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"
#include "common/utils/file/file.h"
#include "pnc/map/map_lib.h"
#include <bits/stdc++.h>

namespace vz {

const double eps = 1e-2;
const double alpha = 5;
const double beta = 0;
const double throttle_table[81] = {0,0.00740,0.01396,0.01975,0.02484,0.02930,0.03317,0.03652,0.03938,0.04181,0.04384,0.04551,0.04686,0.04792,0.04871,0.04927,0.10940,0.10353,0.09291,0.11906,0.13142,0.13174,0.14947,0.17174,0.19190,0.21569,0.26078,0.30588,0.35098,0.39608,0.44118,0.48627,0.53137,0.57647,0.63823,0.70000,0.79882,0.89765,0.99647,1.09529,1.19412,1.32588,1.45765,1.58941,1.72118,1.85294,1.92706,2.00118,2.07529,2.14941,2.22353,2.29765,2.37176,2.44588,2.52000,2.59412,2.66823,2.74235,2.81647,2.89059,2.96470,3.01412,3.06353,3.11294,3.16235,3.21176,3.26117,3.31059,3.36000,3.40941,3.45882,3.53706,3.61529,3.69353,3.77176,3.85000,3.92823,4.00647,4.08470,4.16294,4.24117};
const double brake_table[41] = {0,0.02417,0.04835,0.07252,0.09670,0.12087,0.14504,0.16922,0.19339,0.21757,0.24174,0.26591,0.29009,0.31426,0.33844,0.35556,0.36209,0.36863,0.37516,0.38170,0.38824,0.56471,0.74118,1.16471,1.65882,2.15294,2.75294,3.35294,3.96078,4.56863,5.17647,5.78823,6.40000,7.01176,7.62353,8.23529,8.70588,9.17647,9.64705,10.11764,10.58823};
const int num_predict = 11;
const double pi = acos(-1);
const double Alpha = 2.5;
const double Beta = 0;
const double target_velocity = 10;
const double stop_distance = target_velocity * target_velocity * 0.125;
const double safe_radius_car = 2;
const double safe_radius_ped = 3.5;

struct PID {
  double Kp, Ki, Kd;
  double et_last, et_integral;
  bool is_first_iter;
  PID(double KP, double KI, double KD): Kp(KP), Ki(KI), Kd(KD) {
    clear();
  }
  double calc(double et) {
    et_integral += et * 0.01;
    if (is_first_iter) {
      et_last = et;
      is_first_iter = false;
    }
    double ut = et + Ki * et_integral + Kd * (et - et_last) * 100;
    et_last = et;
    return ut;
  }
  void clear() {
    is_first_iter = true;
    et_integral = 0;
  }
};

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
  q.set_z(0);
  return q;
}

interface::geometry::Vector3d Point2DtoVector3d(const interface::geometry::Point2D& p) {
  interface::geometry::Vector3d q;
  q.set_x(p.x());
  q.set_y(p.y());
  q.set_z(0);
  return q;
}

interface::geometry::Vector3d operator+(const interface::geometry::Vector3d& p,
  const interface::geometry::Vector3d& q) {
  interface::geometry::Vector3d r;
  r.set_x(p.x() + q.x());
  r.set_y(p.y() + q.y());
  r.set_z(p.z() + q.z());
  return r;
}

interface::geometry::Vector3d operator-(const interface::geometry::Vector3d& p,
  const interface::geometry::Vector3d& q) {
  interface::geometry::Vector3d r;
  r.set_x(p.x() - q.x());
  r.set_y(p.y() - q.y());
  r.set_z(p.z() - q.z());
  return r;
}

interface::geometry::Vector3d operator*(const double& a,
  const interface::geometry::Vector3d& q) {
  interface::geometry::Vector3d r;
  r.set_x(a * q.x());
  r.set_y(a * q.y());
  r.set_z(a * q.z());
  return r;
}

interface::geometry::Point3D operator+(const interface::geometry::Point3D& p,
  const interface::geometry::Vector3d& q) {
  interface::geometry::Point3D r;
  r.set_x(p.x() + q.x());
  r.set_y(p.y() + q.y());
  r.set_z(p.z() + q.z());
  return r;
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

class vzVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit vzVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& /* agent_status */) {
    std::string filepath = file::path::Join(file::path::GetProjectRootPath(), "pnc", "agents", "vz", "processed_map_proto.txt");
    file::ReadTextFileToProto(filepath, &map);
  }

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {

    /* a new trip */
    //std::cerr << name() << " " << go_after_red << " " << stop_obs << " " << phase << " " << countdown << "\n";
    if (iters == 0 ||
        distance(agent_status.route_status().destination(), destination) > eps ||
        (go_after_red && !stop_obs && (phase == 4 || phase == 5))) {
      destination = agent_status.route_status().destination();
      if (phase < 4)
        FindRoute(agent_status);
      longitudinal.clear();
      lateral.clear();
      phase = 0;
    }

    /* update current position on the planned trip */
    while (route_pnt.size() > 1 && 
      (VeryClose(route_pnt[0], route_pnt[1]) || 
      CalcDistance(agent_status.vehicle_status().position(), Point2Dto3D(route_pnt[0]))
      > CalcDistance(agent_status.vehicle_status().position(), Point2Dto3D(route_pnt[1])))) {
      //std::cerr << "popped out (" << route_pnt[0].x() << "," << route_pnt[0].y() << ")\n";
      route_pnt.pop_front();
      route_pnt_id.pop_front();
    }

    /* when start, find types of each traffic light */
    double cur_time = agent_status.simulation_status().simulation_time();
    if (3 < cur_time && cur_time < 3.02) {
      lightpattern.resize(map.lane_size());
      for (int i = 0; i < map.lane_size(); ++i)
        lightpattern[i] = -1;
      for (int i = 0; i < agent_status.perception_status().traffic_light_size(); ++i) {
        interface::perception::PerceptionTrafficLightStatus lights = agent_status.perception_status().traffic_light(i);
        for (int j = 0; j < lights.single_traffic_light_status_size(); ++j){
          interface::perception::SingleTrafficLightStatus light = lights.single_traffic_light_status(j);
          for (int k = 0; k < map.lane_size(); ++k)
            if (to_lane_id(light.id().id()) == map.lane(k).id().id()) {
              switch (light.color()) {
                case interface::map::Bulb::RED:
                  lightpattern[k] = 0;
                  break;
                case interface::map::Bulb::GREEN:
                  lightpattern[k] = 2;
                  break;
              }
            }
        }
      }
    }

    /* calculate status of traffic light on current road */
    int cur_color = 2, nxt_color = 3; // 0: red  1: yellow1  2: green  3: yellow2
    double time_to_end = 999999999;
    if (cur_time > 4 && route_pnt.size() > 0) {
      int cur_id = route_pnt_id[0];
      if (lightpattern[cur_id] > -1) {
        int full_cycle = floor((cur_time - 3) / 46);
        double rem_time = cur_time - 3 - 46 * full_cycle;
        if (rem_time < 20) {
          cur_color = lightpattern[cur_id];
          time_to_end = 20 - rem_time;
        } else if (rem_time < 23) {
          cur_color = lightpattern[cur_id] ^ 1;
          time_to_end = 23 - rem_time;
        } else if (rem_time < 43) {
          cur_color = lightpattern[cur_id] ^ 2;
          time_to_end = 43 - rem_time;
        } else {
          cur_color = lightpattern[cur_id] ^ 3;
          time_to_end = 46 - rem_time;
        }
        nxt_color = (cur_color + 1) % 4;
      }
    }
    go_after_red = (cur_color != 0);

    /* do not rush red light */
    double rem_distance = distance(agent_status.vehicle_status().position(), route_pnt[0]);
    int ii;
    for (ii = 0; ii < route_pnt.size() - 1 && route_pnt_id[ii + 1] == route_pnt_id[0]; ++ii)
      rem_distance += distance(route_pnt[ii], route_pnt[ii + 1]);

    double tar_a, tar_v;
    double acc = CalcVelocity(agent_status.vehicle_status().acceleration_vcs());
    double vel = CalcVelocity(agent_status.vehicle_status().velocity());
    double ds = distance(agent_status.vehicle_status().position(), route_pnt[0]);
    for (int i = 1; i < route_pnt.size(); ++i)
      ds += distance(route_pnt[i - 1], route_pnt[i]);

    //std::cout << iters * 0.01 << " " << vel << "\n";
    //std::cerr << iters * 0.01 << " " << vel << "\n";
    
    //std::cerr << "phase = " << phase << "\n";
    if (vel > 0.1 && phase  <= 1 && ii < route_pnt.size() - 1 && route_pnt_id[ii + 1] != route_pnt_id[0]) {
      double predict_time = rem_distance / vel;
      //std::cerr << name() << " cur_color = " << cur_color << " nxt_color = " << nxt_color << " time_to_end = " << time_to_end << " predict_time = " << predict_time << "\n";
      if (rem_distance < stop_distance + vehicle_params().vehicle_fa_to_front()) {
        if (cur_color == 0) {
          if (predict_time < time_to_end + 3.5)
            phase = 4;
        } else if (nxt_color == 0) {
          if (predict_time > time_to_end - 0.5)
            phase = 4;
        }
      }
    }

    /* seeking nearby obstacles */
    stop_obs = false;
    if (route_pnt.size() > 0 && phase <= 1) {
      for (int i = 0; i < agent_status.perception_status().obstacle_size(); ++i) {
        if (stop_obs) break;
        interface::perception::PerceptionObstacle obstacle = agent_status.perception_status().obstacle(i);
        if (obstacle.type() != interface::perception::ObjectType::CAR && obstacle.type() != interface::perception::ObjectType::PEDESTRIAN)
          continue;
        bool is_car = (obstacle.type() == interface::perception::ObjectType::CAR);
        std::vector <interface::geometry::Point3D> points;
        for (int j = 0; j < obstacle.polygon_point_size(); ++j)
          points.push_back(obstacle.polygon_point(j));
        interface::geometry::Vector3d dir;
        dir.set_x(cos(obstacle.heading()));
        dir.set_y(sin(obstacle.heading()));
        dir.set_z(0);
        double obs_vel = std::max(0.01, obstacle.speed()), safe_r;
        if (obstacle.type() == interface::perception::ObjectType::CAR)
          safe_r = safe_radius_car;
        else
          safe_r = safe_radius_ped;
        double d = distance(agent_status.vehicle_status().position(), route_pnt[0]);
        double t_, lt_ = 0;
        for (int j = 0; j < route_pnt.size(); ++j) {
          if (j > 0)
            d += distance(route_pnt[j - 1], route_pnt[j]);
          if (d > stop_distance + 5)
            break;
          double d_ = d - vehicle_params().vehicle_fa_to_front();
          double t_ = std::min((d_ + vehicle_params().vehicle_length()) / std::max(0.01, vel), vel / 5), dmin = 1e10;
          double dt_ = fmax((t_ - lt_) / 100, 0.1);
          int numt = (t_ - lt_) / dt_;
          for (int k = 0; k < points.size(); ++k)
            for (int y = 0; y <= numt; ++y) {
              double t = lt_ + dt_ * y;
              dmin = std::min(dmin, distance(points[k] + (t * obs_vel) * dir, route_pnt[j]));
              if (is_car) {
                dmin = std::min(dmin, distance(points[k] + (t * fmax(obs_vel - 2, 0.0)) * dir, route_pnt[j]));
                if (fabs(CalcAngle(agent_status.vehicle_status().velocity(), dir)) < 0.5)
                  dmin = std::min(dmin, distance(points[k] + (t * fmax(obs_vel + 2, 0.0)) * dir, route_pnt[j]));
              }
            }
          lt_ = t_;
          if (dmin < safe_r) {
            stop_obs = true;
            break;
          }
        }
      }
    }
    if (stop_obs)
      phase = 5;

    /* automata of controlling */
    switch (phase) {
      case 0: // start
        tar_a = 4;
        tar_v = target_velocity;
        if (vel > target_velocity + eps)
          phase = 1;
        if (ds < stop_distance)
          phase = 2;
        break;
      case 1: // keep velocity
        if (vel > target_velocity + eps) {
          tar_a = -4;
          tar_v = target_velocity;
        } else if (vel < target_velocity - eps) {
          tar_a = 4;
          tar_v = target_velocity;
        } else {
          tar_a = 0;
          tar_v = target_velocity;
        }
        if (ds < stop_distance)
          phase = 2;
        break;
      case 2: // end
        tar_a = -11;
        tar_v = 0;
        if (vel < eps)
          phase = 3;
        break;
      case 3: // tuning
        if (ds > 1) {
          tar_a = 0;
          tar_v = ds * 0.45;
        } else {
          tar_a = -11;
          tar_v = 0;
        }
        break;
      case 4: // red light
        tar_a = -11;
        tar_v = 0;
        break;
      case 5: // obstacle
        tar_a = -11;
        tar_v = 0;
        break;
    }

    //std::cerr << "phase = " << phase << "\n";

    /* longitudinal control */
    double et = acc - tar_a + alpha * (vel - tar_v) + beta * ds;
    double ut = longitudinal.calc(et);
    //std::cerr << name() << " " << et << " " << ut << "\n";
    
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

    /* lateral control */
    if (vel > 1 && route_pnt.size() >= num_predict * 0.7) {
      int k = num_predict < route_pnt.size() ? num_predict : route_pnt.size();
      //for (int t = 0; t < k; ++t)
      //  std::cerr << "(" << route_pnt[t].x() << "," << route_pnt[t].y() << ")";
      //std::cerr << "\n";
      interface::geometry::Point2D Q = route_pnt[k - 1];
      interface::geometry::Vector3d P = agent_status.vehicle_status().position();
      double angle = CalcAngle(agent_status.vehicle_status().velocity(),
        Point2DtoVector3d(Q) - P);
      double L = CalcDistance(P, Point2Dto3D(Q));
      double kappa = 2 * sin(angle) / L;
      double Et = kappa + Alpha * angle + Beta * L;
      double Ut = lateral.calc(Et);
      double steer = CalcSteerFromUt(Ut);
      command.set_steering_angle(steer);
      //std::cerr << kappa << " " << angle << " " << L << " " << Et << " " << Ut << " " << steer << "\n";
    } else
      lateral.calc(0);

    ++iters;
    return command;
  }

 private:
  
  std::string to_lane_id(std::string s) {
    return s.substr(2, s.length() - 5);
  }

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
    double f = (exp(fabs(ut) * 0.5) - 1);
    if (ut < 0)
      f = -f;
    return CalcSteerFromCurv(f);
  }

  double CalcAngle(const interface::geometry::Vector3d& a,
    const interface::geometry::Vector3d& b) {
    //std::cerr << "a = (" << a.x() << ", " << a.y() << ") b = (" << b.x() << "," << b.y() << ")\n";
    
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

  double CalcSteerFromCurv(double kappa) {
    /*
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
    */
    return 44 * kappa;
  }

  void FindRoute(const interface::agent::AgentStatus& agent_status) {
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
    route_pnt_id.clear();

    // starting point and ending point are on the same lane, and they are in the correct direction
    // directly output the route
    if (id_st == id_ed && point_id_st <= point_id_ed) {
      for (int j = point_id_st; j <= point_id_ed; ++j) {
        route_pnt.push_back(Point3Dto2D(map.lane(id_st).central_line().point(j)));
        route_pnt_id.push_back(id_st);
      }
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
    for (int j = point_id_st; j < map.lane(id_st).central_line().point_size(); ++j) {
      route_pnt.push_back(Point3Dto2D(map.lane(id_st).central_line().point(j)));
      route_pnt_id.push_back(id_st);
    }
    for (auto x : route_id) {
      if (x == 2 * id_ed + 2) break;
      if (x & 1) continue;
      int i = (x - 2) / 2;
      interface::map::Lane li = map.lane(i);
      int sn = li.central_line().point_size();
      for (int j = 0; j < sn; ++j) {
        route_pnt.push_back(Point3Dto2D(map.lane(i).central_line().point(j)));
        route_pnt_id.push_back(i);
      }
    }
    for (int j = 0; j <= point_id_ed; ++j) {
      route_pnt.push_back(Point3Dto2D(map.lane(id_ed).central_line().point(j)));
      route_pnt_id.push_back(id_ed);
    }
  }
  
  PID longitudinal = PID(1.0, 0.1, 0.3);
  PID lateral = PID(1.0, target_velocity * 0.135, 1.05);

  interface::map::Map map;
  std::deque <interface::geometry::Point2D> route_pnt;
  std::deque <int> route_pnt_id;
  int iters = 0;

  std::vector <int> lightpattern;
  bool go_after_red;
  bool stop_obs;

  interface::geometry::Point3D destination;
  int phase = 0;

};



}
