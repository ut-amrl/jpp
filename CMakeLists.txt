cmake_minimum_required(VERSION 2.8.3)
project(libjpp)

#set(DAISY_DESC_INC_DIR /usr/local/lib/libdaisy.so)
set(DAISY_DESC_INC_DIR ./src/daisy_descriptor/include)
set(JPP_INC_DIR ./include)
set(CMAKE_CXX_FLAGS "-msse3 -fopenmp")
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)

include_directories(
  ${DAISY_DESC_INC_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${JPP_INC_DIR}
)

file(GLOB DAISY_DESC_SRC_FILES "src/daisy_descriptor/src/*")
file(GLOB JPP_LIB_FILES
  "src/stereo.cc" 
  "src/jpp.cc"
  "src/astarplanner.cc" 
  "src/rrt.cc" 
)
file(GLOB JPP_BIN_FILES
  "src/main.cc"
)

add_library(libjpp SHARED ${DAISY_DESC_SRC_FILES} ${JPP_LIB_FILES})
set_target_properties(libjpp PROPERTIES OUTPUT_NAME "jpp")
target_link_libraries(libjpp ${OpenCV_LIBS} ${OpenMP_LIBS} "-lconfig")

add_executable(jpp ${JPP_BIN_FILES})
target_link_libraries(jpp "-lpopt -lconfig" libjpp)

add_executable(analysis "src/analysis.cc")
target_link_libraries(analysis "-lpopt -lconfig" libjpp)