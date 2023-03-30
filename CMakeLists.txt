cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 17)

project( QUADCONTROL )

find_package(Eigen3  REQUIRED )
find_package(manif  REQUIRED )

file(GLOB QUADCONTROL_SRC
     "include/*.h"
     "src/*.cpp"
)

add_executable( quad ${QUADCONTROL_SRC})
target_include_directories( quad SYSTEM PUBLIC ${manif_INCLUDE_DIRS} )
