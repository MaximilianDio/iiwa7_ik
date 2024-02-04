# CMake minimum version requirement
cmake_minimum_required(VERSION 3.12)

# Project name
project(mars_model)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# ##########
# Options
# ##########
option(PACKAGE_TESTS "Build the tests" ON)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Add include directory
include_directories(include)

# # Add library
# add_library(mars_model_lib src/mars_model.cpp)
# # add library dependencies
# target_link_libraries(mars_model_lib PUBLIC Eigen3::Eigen pinocchio::pinocchio)


# Testing
if(PACKAGE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()