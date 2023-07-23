cmake_minimum_required(VERSION 3.16)

# set the project name and version
project(ppp-cpp)

# Find OSQP library and headers
find_package(osqp REQUIRED)
find_package(glog 0.6.0 REQUIRED)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLE_OUTPUT_PATH /home/ym/workspace/ppp-cpp/bin)

# add the executable
add_executable(ppp-cpp src/ppp.cc src/fem_pos_deviation_osqp_interface.cc src/fem_pos_deviation_smoother.cc)

LINK_DIRECTORIES(/usr/local/lib)

# Link the OSQP shared library
target_link_libraries(ppp-cpp osqp::osqp glog::glog)
