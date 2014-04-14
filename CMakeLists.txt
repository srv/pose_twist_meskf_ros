cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

project(pose_twist_meskf_ros)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nav_msgs
  memsense_imu
  auv_sensor_msgs
  eigen_conversions
  tf
  sensor_msgs
  nodelet
  eigen)


# Add extra compiler warnings:
set(CMAKE_CXX_FLAGS "-Wall -Wextra" ${CMAKE_CXX_FLAGS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions("-DEIGEN_DONT_ALIGN_STATICALLY")

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Boost:
find_package(Boost REQUIRED COMPONENTS signals system)
include_directories(${Boost_INCLUDE_DIRS})

# Eigen library:
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Dependencies - BFL:
# Use script to find BFL. If it's not installed, set G2O_INCLUDE_DIRS manually 
# using -DG2O_INCLUDE_DIRS.
find_package(BFL REQUIRED)
include_directories(${BFL_INCLUDE_DIR})

include_directories(src/filter)

catkin_package()

# Add the Image Hashing library
add_library(pose_twist_meskf 
            src/filter/pose_twist_meskf.cpp
            src/filter/extendedkalmanfilter_resetcapable.cpp
            src/filter/nominal_state_vector.cpp
            src/filter/error_state_vector.cpp
            src/filter/input_vector.cpp
            src/filter/analyticconditionalgaussian_posetwisterrorstate.cpp
            src/filter/analyticconditionalgaussian_errormeasurement.cpp
            src/filter/visual_measurement_vector.cpp
            src/filter/visual_measurement_error_vector.cpp
            src/filter/depth_measurement_error_vector.cpp
            src/filter/depth_measurement_vector.cpp
            src/filter/analyticconditionalgaussian_visualmeasurement.cpp
            src/filter/analyticconditionalgaussian_depthmeasurement.cpp)

add_executable(pose_twist_meskf_node 
               src/nodes/pose_twist_meskf_node.cpp
               src/nodes/pose_twist_meskf_node_base.cpp)

target_link_libraries(pose_twist_meskf_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${BFL_LIBRARIES} pose_twist_meskf)