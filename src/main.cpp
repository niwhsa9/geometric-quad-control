#include <iostream>
#include <manif/manif.h>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <iostream>
#include <memory>

using namespace webots;

int main() {
  auto robot = std::make_unique<Robot>();
  auto gyro = robot->getGyro("gyro");

  while (robot->step(32) != -1) {
    auto v = gyro->getValues();
    std::cout << "Hello World! " << v[0] << " " << v[1] << " "<< std::endl;
  }

  return 0;
}