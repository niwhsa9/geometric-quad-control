#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <manif/manif.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Accelerometer.hpp>
#include <iostream>
#include <memory>
#include "ekf.h"
#include "teleop.h"

using namespace webots;

Eigen::Vector3d get_gps() {

}
Eigen::Vector3d get_true_pos() {

}

int main() {
  auto robot = std::make_unique<Robot>();
  auto gyro = robot->getGyro("gyro");
  auto accel = robot->getAccelerometer("accelerometer");
  auto gps = robot->getGPS("gps");

  Teleop teleop(robot.get());
  auto dt = robot->getBasicTimeStep();

  gps->enable(dt);
  accel->enable(dt);

  EKF ekf(
    EKF::ProcNoiseMat::Identity(), 
    EKF::ObvNoiseMatAccel::Identity(),
    EKF::ObvNoiseMatGPS::Identity() * 0.1,
    dt/1000.0
    );

  int iter_cnt = 0;

  while (robot->step(dt) != -1) {
    auto v = gyro->getValues();
    auto a = accel->getValues();
    auto g = gps->getValues();
    auto g_v = gps->getSpeedVector();
    
    // robot teleop
    teleop.keyboard_ctrl();

    // TODO skip start iterations due to strange contact forces at init in sim
    if(iter_cnt > 1500 && !isnan(a[0]) && !isnan(g[0])) {
      ekf.predict(Eigen::Vector3d(v), Eigen::Vector3d(a));
      ekf.update_gps(Eigen::Vector3d(g), Eigen::Vector3d(g_v));
      auto state = ekf.get_state();
      std::cout << "ekf " << state.x() << " " << state.y() <<  " " << state.z()  << " "<< "truth " << g[0] << " " << g[1] <<  " " << g[2] << " "<< std::endl;
    }

    //std::cout << "ekf  " << state.X.x() << " " << state.X.y() <<  " " << state.X.z()  << " "<< std::endl
    //std::cout << "rot  " << state.X.quat().x() << " "<< state.X.quat().y() << " "<< state.X.quat().z() << " "  << state.X.quat().w() << std::endl;
    //std::cout << "truth  " << g[0] << " " << g[1] <<  " " << g[2] << " "<< std::endl;
    iter_cnt++; 
    
  }

  return 0;
}