cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Add extra compiler warnings:
set(CMAKE_CXX_FLAGS "-Wall -Wextra" ${CMAKE_CXX_FLAGS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

add_definitions("-DEIGEN_DONT_ALIGN_STATICALLY")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Eigen library:
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library(pose_twist_meskf 
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

include_directories(src/filter)

rosbuild_add_executable(pose_twist_meskf_node 
                          src/nodes/pose_twist_meskf_node.cpp
                          src/nodes/pose_twist_meskf_node_base.cpp)
target_link_libraries(pose_twist_meskf_node
                        pose_twist_meskf)
rosbuild_link_boost(pose_twist_meskf_node signals)

rosbuild_add_library(pose_twist_meskf_nodelet
                       src/nodes/pose_twist_meskf_nodelet.cpp
                       src/nodes/pose_twist_meskf_node_base.cpp)
target_link_libraries(pose_twist_meskf_nodelet
                        pose_twist_meskf)

rosbuild_add_executable(test_sfly EXCLUDE_FROM_ALL 
                          test/test_sfly_main.cpp)
target_link_libraries(test_sfly
                        pose_twist_meskf)
