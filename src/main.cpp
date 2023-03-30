#include <iostream>
#include <manif/manif.h>

int main() {
   auto state = manif::SE3d::Identity(); 
   std::cout << "test " << state << std::endl;

}
