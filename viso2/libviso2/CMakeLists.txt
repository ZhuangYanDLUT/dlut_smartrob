cmake_minimum_required(VERSION 2.8.3)
project(libviso2)

find_package(catkin REQUIRED )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
#set(CMAKE_CXX_FLAGS "-msse3")
add_definitions(-msse3)
add_library(viso2
  libviso2/src/filter.cpp
  libviso2/src/matcher.cpp
  libviso2/src/matrix.cpp
  libviso2/src/reconstruction.cpp
  libviso2/src/triangle.cpp
  libviso2/src/viso.cpp
  libviso2/src/viso_mono.cpp
  libviso2/src/viso_stereo.cpp
  libviso2/src/P3p.cpp)

include_directories(libviso2/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS libviso2/src
  LIBRARIES viso2
)
