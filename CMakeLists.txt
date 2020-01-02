cmake_minimum_required(VERSION 3.0)
project(reemc_balance_control)


set ( CMAKE_CXX_STANDARD 14 )
set ( CMAKE_EXPORT_COMPILE_COMMANDS ON )
# set ( CMAKE_VERBOSE_MAKEFILE ON )
# set ( CMAKE_BUILD_TYPE Debug )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  realtime_tools
  controller_interface
  kdl_parser
)

find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${RBDL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES ${PROJECT_NAME}
    # CATKIN_DEPENDS roscpp realtime_tools controller_interface
    # DEPENDS Eigen3 RBDL
)

add_library(
    ${PROJECT_NAME}
    src/zmp_controller.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${RBDL_LIBRARY}
    )