cmake_minimum_required(VERSION 2.8.3)
project(eigen_typekit)

# Dependencies

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

orocos_typekit(eigen_typekit eigen_typekit.cpp)

orocos_install_headers(eigen_typekit.hpp)

orocos_generate_package()
