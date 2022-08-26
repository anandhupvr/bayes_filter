cmake_minimum_required(VERSION 3.0.2)
project(bayes_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bayes_filter
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS system_lib
 DEPENDS Eigen
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(pf_test 
              src/test_node.cpp
              src/Particle.cpp
              src/Map.cpp
              src/ParticleFilter.cpp
              src/MotionModel.cpp
              src/SensorModel.cpp
                )
# target_link_libraries(main ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${LIBRARIES})
target_link_libraries(pf_test
                      ${roscpp_LIBRARIES}
                      ${std_msgs_LIBRARIES}
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
                      )