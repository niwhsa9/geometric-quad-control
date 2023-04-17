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
#include "simulator.h"
#include <random>

using namespace webots;

int main() {
  SimQuadcopter quad;

  EKFWorker ekf(
    EKF::ProcNoiseMat::Identity()*0.001, 
    EKF::ObvNoiseMatAccel::Identity() * 0.001,
    EKF::ObvNoiseMatGPS::Identity() * 0.1,
    quad.get_dt()/1000.0
    );

  ekf.loop_ekf();

  while (true) {
    quad.step_sim();

    // robot teleop
    quad.keyboard_ctrl();

    auto omega = quad.get_gyro();
    auto a = quad.get_accel();
    auto gps_pos = quad.get_pos();
    auto gps_vel = quad.get_vel();
    auto mag = quad.get_mag();

    auto cheater_pos = quad.get_pos_true();
    auto cheater_vel = quad.get_vel_true();
    auto cheater_rot = quad.get_rot_true();

    // TODO skip start iterations due to strange contact forces at init in sim
    if(!isnan(a.x()) && !isnan(gps_pos.x())) {
      //omega.setZero();
      ekf.predict(omega, a);
      auto state = ekf.get_state();

      auto rot_delta = ekf.get_state().asSO3().between(manif::SO3d(cheater_rot));
      ekf.update_imu(mag, a);
      ekf.update_gps(gps_pos, gps_vel);

      //std::cout << "rot error " << rot_delta.log().weightedNorm() << std::endl;
      //"truth " << gps_pos.x() << " " << gps_pos.y() <<  " " << gps_pos.z() << " "<< std::endl;
      //std::cout << "pos error " << (gps_pos - ekf.get_state().translation()).norm() << std::endl;
      //std::cout << "vel error" << (gps_vel - ekf.get_state().linearVelocity()).norm() << std::endl;
      std::cout << "ekf " << state.x() << " " << state.y() <<  " " << state.z()  << " "<< 
        "truth " << gps_pos.x() << " " << gps_pos.y() <<  " " << gps_pos.z() << " "<< std::endl;

    }
  }

  return 0;
}