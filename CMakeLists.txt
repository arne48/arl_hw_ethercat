cmake_minimum_required(VERSION 3.2.0)
project(arl_hw_ethercat)
include(ExternalProject)
set (CMAKE_CXX_STANDARD 11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arl_hw_ethercat
#  CATKIN_DEPENDS roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

ExternalProject_Add(soem-lib
        GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM
        GIT_TAG master
        PATCH_COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/patches/build_soem_shared.patch
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX})

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
)

add_executable(arl_ethercat_test src/ethercat_master_test.cpp)
add_dependencies(arl_ethercat_test soem-lib)
target_link_libraries(arl_ethercat_test
        ${catkin_LIBRARIES}
        ${CATKIN_DEVEL_PREFIX}/lib/libsoem.so
        )