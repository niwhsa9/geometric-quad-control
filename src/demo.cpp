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
#include "controller.h"
#include <random>

using namespace webots;

int main() {
  SimQuadcopter quad;
  EKFWorker ekf(
    EKF::ProcNoiseMat::Identity()*0.0005, 
    EKF::ObvNoiseMatAccel::Identity() * 0.1,
    EKF::ObvNoiseMatGPS::Identity() * 0.1,
    EKF::ObvNoiseMatMag::Identity() * 0.05,
    quad.get_dt()/1000.0
    );
  ekf.loop_ekf();

  Controller ctrl;
  double x, y, z, yaw = 0.0;
  double dx, dy, dz = 0.0;

  while (true) {
    quad.step_sim();

    // robot teleop
    //quad.keyboard_ctrl();

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
      ekf.predict(omega, a);
      //Eigen::Vector3d fake_acc = cheater_rot.inverse().toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, 9.81);
      Eigen::Vector3d fake_acc = a - Eigen::Vector3d(0.0, 0.0, ctrl.get_prev_az() - 9.81);//cheater_rot.inverse().toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, 9.81);
      ekf.update_imu(mag, fake_acc);
      ekf.update_gps(gps_pos, gps_vel);

      auto state = ekf.get_state();
      auto cheater_state = manif::SE_2_3d(cheater_pos, cheater_rot, cheater_vel);

      manif::SE_2_3d des
        (Eigen::Vector3d(0.0, 0.0, 5.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d::Zero());

      Eigen::Vector3d accel_in_body = a - state.rotation().transpose()*Eigen::Vector3d(0.0, 0.0, 9.81);
      //Eigen::Vector4d cmd = ctrl.iterate_ctrl(Controller::State{state, omega, accel_in_body}, 
        //Controller::State{des, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});

      double time = quad.get_time();
      
      if(time < 10) {
        z = quad.get_time() * 0.5;
        dz = 0.5;
      } else {
        double t = (quad.get_time()-10);
        x =  t * 0.5;
        dx = 0.5;
        double T = 3.14/2;
        y = 1.2 * sin(T*t);
        dy = 1.2 * T * cos(T*t);
        z = 5 + 1.2 * cos(T*t)-1;
        dz = -1.2 * sin(T*t) * T;
        yaw = 0.0;//t * 0.5;
      }

      Controller::FlatOutput d_o{Eigen::Vector3d(x, y, z), Eigen::Vector3d(dx, dy, dz),
        Eigen::Vector3d::Zero(), yaw};
      Controller::State cur_state{state, omega, accel_in_body};

      Eigen::Vector4d cmd = ctrl.track_target(d_o, cur_state, std::nullopt);

      quad.set_vel(cmd);
      //auto rot_delta = ekf.get_state().asSO3().between(manif::SO3d(cheater_rot));
      //std::cout << "rot error " << rot_delta.log().weightedNorm() << std::endl;
      //"truth " << gps_pos.x() << " " << gps_pos.y() <<  " " << gps_pos.z() << " "<< std::endl;
      //std::cout << "pos error " << (gps_pos - ekf.get_state().translation()).norm() << std::endl;
      //std::cout << "vel error" << (gps_vel - ekf.get_state().linearVelocity()).norm() << std::endl;
      /*
      std::cout << "ekf " << state.x() << " " << state.y() <<  " " << state.z()  << " "<< 
        "truth " << gps_pos.x() << " " << gps_pos.y() <<  " " << gps_pos.z() << " "<< std::endl;
      */
    }
  }

  return 0;
}