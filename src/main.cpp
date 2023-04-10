#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <manif/manif.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Compass.hpp>
#include <iostream>
#include <memory>
#include "ekf.h"
#include "teleop.h"
#include <random>

using namespace webots;

Eigen::Vector3d add_noise(Eigen::Vector3d vec, std::normal_distribution<double> &nd, std::mt19937 &gen) {
    vec.x() += nd(gen);
    vec.y() += nd(gen);
    vec.z() += nd(gen);
    return vec;
}

int main() {
  auto robot = std::make_unique<Robot>();
  auto gyro = robot->getGyro("gyro");
  auto accel = robot->getAccelerometer("accelerometer");
  auto gps = robot->getGPS("gps");
  auto compass = robot->getCompass("compass");
  auto inertial = robot->getCompass("inerital unit");

  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::normal_distribution<double> nd_gps_pos(0.0, 0.1); 
  std::normal_distribution<double> nd_gps_vel(0.0, 0.04); 

  Teleop teleop(robot.get());
  auto dt = robot->getBasicTimeStep();

  gps->enable(dt);
  accel->enable(dt);

  EKF ekf(
    EKF::ProcNoiseMat::Identity()*0.001, 
    EKF::ObvNoiseMatAccel::Identity(),
    EKF::ObvNoiseMatGPS::Identity() * 0.1,
    dt/1000.0
    );

  int iter_cnt = 0;

  while (robot->step(dt) != -1) {
    Eigen::Vector3d omega(gyro->getValues());
    Eigen::Vector3d a(accel->getValues());
    Eigen::Vector3d gps_pos(gps->getValues());
    Eigen::Vector3d gps_vel(gps->getSpeedVector());

    Eigen::Vector3d noisy_gps_pos = add_noise(gps_pos, nd_gps_pos, gen);
    Eigen::Vector3d noisy_gps_vel = add_noise(gps_vel, nd_gps_vel, gen);
    
    // robot teleop
    teleop.keyboard_ctrl();

    // TODO skip start iterations due to strange contact forces at init in sim
    if(iter_cnt > 1500 && !isnan(a.x()) && !isnan(gps_pos.x())) {
      ekf.predict(omega, a);
      ekf.update_gps(noisy_gps_pos, noisy_gps_vel);
      auto state = ekf.get_state();
      /*
      std::cout << "ekf " << state.x() << " " << state.y() <<  " " << state.z()  << " "<< 
        //"truth " << gps_pos.x() << " " << gps_pos.y() <<  " " << gps_pos.z() << " "<< std::endl;
        "truth " << noisy_gps_pos.x() << " " << noisy_gps_pos.y() <<  " " << noisy_gps_pos.z() << " "<< std::endl;
      */
    }

    iter_cnt++; 
    
  }

  return 0;
}