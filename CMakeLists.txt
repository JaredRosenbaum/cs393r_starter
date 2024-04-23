PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# SET(CMAKE_BUILD_TYPE Debug)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror") # !!!CHANGED FROM 11!!!

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags amrl-shared-lib boost_system lua5.1
    pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

ROSBUILD_ADD_EXECUTABLE(slam
                        src/slam/slam_main.cc
                        src/slam/slam.cc
                        src/slam/icp.cpp)
TARGET_LINK_LIBRARIES(slam shared_library gtsam ${libs})


ROSBUILD_ADD_EXECUTABLE(particle_filter
                        src/particle_filter/particle_filter_main.cc
                        src/particle_filter/particle_filter.cc)
TARGET_LINK_LIBRARIES(particle_filter shared_library ${libs})

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc
                        src/navigation/controllers.cpp
                        src/navigation/global_planner.cc
                        src/navigation/functions.cpp
                        src/navigation/path_generation.cpp
                        src/navigation/local_planner.cpp)
TARGET_LINK_LIBRARIES(navigation shared_library ${libs})

ADD_EXECUTABLE(eigen_tutorial
               src/eigen_tutorial.cc)

ADD_EXECUTABLE(simple_queue_test
               src/navigation/simple_queue_test.cc)

ADD_EXECUTABLE(cimg_example
               src/cimg_example.cc)
TARGET_LINK_LIBRARIES(cimg_example shared_library ${libs} X11)

ADD_EXECUTABLE(resampling_test
               src/particle_filter/resampling_test.cpp)
TARGET_LINK_LIBRARIES(resampling_test shared_library ${libs})

ADD_EXECUTABLE(rasterization_test
               src/slam/rasterization_test.cpp)
TARGET_LINK_LIBRARIES(rasterization_test shared_library ${libs})

# set(CMAKE_PREFIX_PATH "/home/dev/gtsam/local:${CMAKE_PREFIX_PATH}")
# find_package(GTSAMCMakeTools)
# find_package(GTSAM REQUIRED)
# include_directories(${GTSAM_INCLUDE_DIR})
add_executable(gtsam_test src/slam/gtsam_test.cpp)
target_link_libraries(gtsam_test gtsam)

ADD_EXECUTABLE(motion_model_test
               src/slam/motion_model_test.cpp)
TARGET_LINK_LIBRARIES(motion_model_test shared_library ${libs})

ADD_EXECUTABLE(slam_processor_test
               src/slam/slam_processor_test.cpp
               src/slam/slam_processor)
TARGET_LINK_LIBRARIES(slam_processor_test shared_library ${libs})

ADD_EXECUTABLE(icp_test
               src/slam/icp_test.cpp
               src/slam/icp.cpp)
