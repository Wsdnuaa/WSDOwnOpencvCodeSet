cmake_minimum_required( VERSION 2.8 )
project( myPnp )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)


add_executable( myPnp  ./src/LMPnp.cc )
target_link_libraries(myPnp   ${OpenCV_LIBS} Sophus::Sophus )