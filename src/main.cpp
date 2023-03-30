#include <iostream>
#include <manif/manif.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <memory>

using namespace webots;

int main() {
  auto robot = std::make_unique<Robot>();
  //auto gyro = robot->getGyro("gyro");
  auto gps = robot->getGPS("gps");
  gps->enable(32);

  while (robot->step(32) != -1) {
    //auto v = gyro->getValues();
    auto v = gps->getValues();
    
    //std::cout << "Hello World! " << v[0] << " " << v[1] <<  " " << v[2] << " "<< std::endl;
  }

  return 0;
}