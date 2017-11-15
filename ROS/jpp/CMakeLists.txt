cmake_minimum_required(VERSION 2.8.3)
project(jpp)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()
set(CMAKE_CXX_FLAGS "-march=native -O3 -Wall ${CMAKE_CXX_FLAGS}")

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

set(JPP_LIBS ${PROJECT_SOURCE_DIR}/../../lib/libjpp.so)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set(DAISY_DESC_INC_DIR ${CMAKE_SOURCE_DIR}/../../src/daisy_descriptor/include)
set(LIBJPP_INC_DIR ${CMAKE_SOURCE_DIR}/../../include)
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
  ${DAISY_DESC_INC_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${LIBJPP_INC_DIR}
)

rosbuild_add_executable(navigation
  src/navigation.cpp
)
target_link_libraries(navigation ${JPP_LIBS} config popt)
