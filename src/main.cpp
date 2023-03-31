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
  gps->enable(32);
  accel->enable(32);

  Teleop teleop(robot.get());
  auto dt = robot->getBasicTimeStep();

  EKF ekf(
    EKF::ProcNoiseMat::Identity(), 
    EKF::ObvNoiseMatAccel::Identity(),
    EKF::ObvNoiseMatGPS::Identity(),
    dt/1000.0
    );

  while (robot->step(dt) != -1) {
    auto v = gyro->getValues();
    auto a = accel->getValues();
    auto g = gps->getValues();
    
    // robot teleop
    teleop.keyboard_ctrl();
    if(!isnan(a[0]))
      ekf.predict(Eigen::Vector3d(v), Eigen::Vector3d(a));
    auto state = ekf.get_state();

    std::cout << "measure  " << state.X.x() << " " << state.X.z() <<  " " << state.X.y()  << " "<< std::endl;
    std::cout << "truth  " << g[0] << " " << g[1] <<  " " << g[2] << " "<< std::endl;
    
    
  }

  return 0;
}