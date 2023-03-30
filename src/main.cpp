#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <manif/manif.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <memory>
#include "ekf.h"

using namespace webots;

Eigen::Vector3d get_gps() {

}
Eigen::Vector3d get_true_pos() {

}

int main() {
  auto robot = std::make_unique<Robot>();
  //auto gyro = robot->getGyro("gyro");
  auto gps = robot->getGPS("gps");
  auto truth = robot->getGPS("truth");
  gps->enable(32);

  while (robot->step(32) != -1) {
    //auto v = gyro->getValues();
    auto v = gps->getValues();
    auto g = gps->getValues();
    
    std::cout << "measure " << v[0] << " " << v[1] <<  " " << v[2] << " "<< std::endl;
    std::cout << "truth  " << g[0] << " " << g[1] <<  " " << g[2] << " "<< std::endl;
  }

  return 0;
}