#include <bits/stdc++.h>

#include "homework6/simulation/simulation_world.h"
#include "homework6/simulation/simulation_system.h"
#include "homework6/simulation/dynamic_lib/lib_vehicle_status_model_solver.h"
#include "common/utils/file/file.h"
#include "homework6/proto/simulation_config.pb.h"
#include "common/proto/agent_status.pb.h"
#include "common/proto/map.pb.h"

double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y()) + math::Sqr(velocity.z());
    return std::sqrt(sqr_sum);
}

int main() {
    freopen("/home/vectorzhou/AutoDriving/homework6/gen_table/test.txt", "w", stdout);
    const std::string
        map_path      = "/home/vectorzhou/AutoDriving/homework6/map/grid2/map_proto.txt",
        route_path    = "/home/vectorzhou/AutoDriving/homework6/data/routes/simulation_config_1.txt",
        car_para_path = "/home/vectorzhou/AutoDriving/common/data/vehicle_params/vehicle_params.txt";
    
    interface::homework6::SimulationConfig simulation_config;
    file::ReadTextFileToProto(route_path, &simulation_config);

    interface::map::Map map;
    file::ReadTextFileToProto(map_path, &map);
    
    interface::vehicle::VehicleParams vehicle_params;
    file::ReadTextFileToProto(car_para_path, &vehicle_params);


for (double x = 0.01; x < 1; x += 0.01) {
    interface::agent::AgentStatus agent_status;

  agent_status.mutable_vehicle_status()->mutable_position()->set_x(
      simulation_config.route().start_point().x());
  agent_status.mutable_vehicle_status()->mutable_position()->set_y(
      simulation_config.route().start_point().y());
  agent_status.mutable_vehicle_status()->mutable_position()->set_z(0);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_x(
      simulation_config.route().start_orientation().x());
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_y(
      simulation_config.route().start_orientation().y());
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_z(
      simulation_config.route().start_orientation().z());
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_w(
      simulation_config.route().start_orientation().w());
  agent_status.mutable_route_status()->mutable_destination()->set_x(
      simulation_config.route().end_point().x());
  agent_status.mutable_route_status()->mutable_destination()->set_y(
      simulation_config.route().end_point().y());
  agent_status.mutable_route_status()->mutable_destination()->set_z(0);
  agent_status.mutable_simulation_status()->set_is_alive(true);

    std::unique_ptr<vehicle_status_model::VehicleStatusModelSolver> solver =
      vehicle_status_model::CreateVehicleStatusModelSolver(vehicle_params);
    solver->Initialize(0, agent_status.vehicle_status());
 
  int cnt = 0;
  double sum = 0;
  double current_time_ = 0;
    for (; current_time_ < 20; current_time_ += 0.01) {
        interface::control::ControlCommand command;
        command.set_throttle_ratio(1);
        agent_status.mutable_vehicle_status()->CopyFrom(
          solver->UpdateVehicleStatus(current_time_, command));
    }

    for (; current_time_ < 40; current_time_ += 0.01) {
        interface::control::ControlCommand command;
        command.set_steering_angle(x);
        agent_status.mutable_vehicle_status()->CopyFrom(
          solver->UpdateVehicleStatus(current_time_, command));
    }

    for (; current_time_ < 60; current_time_ += 0.01) {
        interface::control::ControlCommand command;
        command.set_steering_angle(x);
        agent_status.mutable_vehicle_status()->CopyFrom(
          solver->UpdateVehicleStatus(current_time_, command));
        //std::cout << CalcVelocity(agent_status.vehicle_status().angular_velocity_vcs()) << " " 
        //  << CalcVelocity(agent_status.vehicle_status().velocity()) / CalcVelocity(agent_status.vehicle_status().angular_velocity_vcs()) << "\n";        
        sum += CalcVelocity(agent_status.vehicle_status().velocity()) / CalcVelocity(agent_status.vehicle_status().angular_velocity_vcs());
        ++cnt;
    }
  printf("%.2lf %.10lf %10lf\n", x, sum / cnt, x * sum / cnt);
}

    return 0;
}
