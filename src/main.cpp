#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
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
  auto inertial = robot->getInertialUnit("inertial unit");

  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::normal_distribution<double> nd_gps_pos(0.0, 0.1); 
  std::normal_distribution<double> nd_gps_vel(0.0, 0.04); 

  Teleop teleop(robot.get());
  auto dt = robot->getBasicTimeStep();

  gps->enable(dt);
  accel->enable(dt);
  inertial->enable(dt);
  compass->enable(dt);

  EKF ekf(
    EKF::ProcNoiseMat::Identity()*0.001, 
    EKF::ObvNoiseMatAccel::Identity() * 0.001,
    EKF::ObvNoiseMatGPS::Identity() * 0.1,
    dt/1000.0
    );

  int iter_cnt = 0;

  while (robot->step(dt) != -1) {
    Eigen::Vector3d omega(gyro->getValues());
    Eigen::Vector3d a(accel->getValues());
    auto gps_pos_raw = gps->getValues();
    Eigen::Vector3d gps_pos(gps_pos_raw[0], gps_pos_raw[1], gps_pos_raw[2]);
    //Eigen::Vector3d gps_pos(gps_pos_raw);
    auto gps_vel_raw = gps->getSpeedVector();
    Eigen::Vector3d gps_vel(gps_vel_raw[0], gps_vel_raw[1], gps_vel_raw[2]);
    //Eigen::Vector3d gps_vel(gps_vel_raw);

    auto q = inertial->getQuaternion(); // TODO quaternion is in wrong frame
    Eigen::Quaterniond q_truth(q[3], q[0], q[1], q[2]);
    manif::SO3d rot_truth(q_truth);
    //rot_truth  = manif::SO3Tangentd(Eigen::Vector3d(0, 0, 3.14159/2)).plus(rot_truth);
    //rot_truth  = rot_truth + manif::SO3Tangentd(Eigen::Vector3d(0, 0, 3.14159/2));
    //rot_truth

    Eigen::Vector3d mag(compass->getValues());

    Eigen::Vector3d noisy_gps_pos = add_noise(gps_pos, nd_gps_pos, gen);
    Eigen::Vector3d noisy_gps_vel = add_noise(gps_vel, nd_gps_vel, gen);
    
    // robot teleop
    teleop.keyboard_ctrl();

    // TODO skip start iterations due to strange contact forces at init in sim
    if(iter_cnt > 0 && !isnan(a.x()) && !isnan(gps_pos.x())) {
      //omega.setZero();
      ekf.predict(omega, a);
      ekf.update_gps(noisy_gps_pos, noisy_gps_vel);
      auto state = ekf.get_state();

      auto rot_delta = ekf.get_state().asSO3().between(rot_truth);
      ekf.update_imu(mag, a);
      //std::cout << "mag " << mag << std::endl;
      //std::cout << "expected " << rot_truth.rotation().inverse() * Eigen::Vector3d(0, 1, 0) << std::endl;
      //std::cout << "gps " << gps_pos << std::endl;
      //std::cout << "rot error " << rot_delta.log().weightedNorm() << std::endl;
      //std::cout << "euler "<< state.rotation().eulerAngles(2, 1, 0) << std::endl;
      //std::cout << "truth "<< rot_truth.rotation().eulerAngles(2, 1, 0) << std::endl;
      //std::cout << "rot truth " << rot_truth.coeffs() << std::endl;
      //std::cout << "mag " << mag << std::endl;
      //std::cout << "gps " << gps_pos << std::endl;

      //std::cout << "rot truth " << rot_truth.coeffs()  << std::endl;
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