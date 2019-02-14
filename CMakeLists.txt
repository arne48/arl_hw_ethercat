cmake_minimum_required(VERSION 3.2.0)
project(arl_hw_ethercat)
include(ExternalProject)
set (CMAKE_CXX_STANDARD 14)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall")

find_package(catkin REQUIRED COMPONENTS
        control_toolbox
        controller_manager
        diagnostic_updater
        hardware_interface
        arl_controllers
        arl_interfaces
        arl_hw_msgs
        realtime_tools
        roscpp
        std_srvs
)

find_package (Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arl_hw_ethercat
  CATKIN_DEPENDS control_toolbox controller_manager hardware_interface roscpp
#  DEPENDS system_lib
)

ExternalProject_Add(soem-lib
        GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM
        GIT_TAG 4427684cc553d3b8a2b6c675693097cc12a424e4
        PATCH_COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/patches/make_shared.patch
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX})

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
)

add_library(arl_robot src/robot.cpp src/ethercat_master.cpp)
target_link_libraries(arl_robot ${catkin_LIBRARIES} ${CATKIN_DEVEL_PREFIX}/lib/libsoem.so)

add_executable(arl_driver_node src/driver.cpp src/driver_utils.cpp src/rt_history.cpp)
target_link_libraries(arl_driver_node
        rt
        arl_robot
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)


add_executable(arl_ethercat_test tests/ethercat_master_test.cpp)
add_dependencies(arl_ethercat_test soem-lib)
target_link_libraries(arl_ethercat_test
        ${catkin_LIBRARIES}
        ${CATKIN_DEVEL_PREFIX}/lib/libsoem.so
)
