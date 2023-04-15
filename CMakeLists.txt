cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 20)
set(WEBOTS_HOME "/usr/local/webots")

project( QUADCONTROL )

find_package(Eigen3  REQUIRED )
find_package(manif  REQUIRED )

file(GLOB QUADCONTROL_SRC
     "include/*.h"
     "src/*.cpp"
)

link_directories(${WEBOTS_HOME}/lib)
add_executable( quad ${QUADCONTROL_SRC} )
target_link_libraries(quad ${WEBOTS_HOME}/lib/controller/libCppController.so)
target_include_directories( quad SYSTEM PUBLIC ${manif_INCLUDE_DIRS} ${WEBOTS_HOME}/include/controller/cpp include/)
