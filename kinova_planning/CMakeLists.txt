cmake_minimum_required(VERSION 2.8.3)
project(kinova_planning)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs 
    actionlib
    moveit_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    trac_ik_lib
    joint_trajectory_generator
    kinova_driver
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
  FILES
  SolveQP.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    actionlib moveit_msgs message_runtime
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    trac_ik_lib
    kinova_driver
  DEPENDS
    EIGEN3
    orocos_kdl
)


include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(base_planning src/base_planning.cpp)
target_link_libraries(base_planning
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)