cmake_minimum_required(VERSION 2.8.3)
project(gnssimu)


find_package(Eigen3 REQUIRED)

include_directories(include)

#glob files together
file(GLOB_RECURSE SRC_FILES src/main.cpp src/gnssimu.cpp)

add_executable(main ${SRC_FILES})

install(TARGETS main DESTINATION built_executables)