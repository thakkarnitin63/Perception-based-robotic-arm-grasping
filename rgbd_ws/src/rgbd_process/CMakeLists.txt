cmake_minimum_required(VERSION 3.0.2)
project(rgbd_process)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)


catkin_package(
 LIBRARIES 
 INCLUDE_DIRS
 CATKIN_DEPENDS 
    roscpp
    rospy 
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})



add_subdirectory(doc/controller_configuration)
add_subdirectory(doc/hand_eye_calibration)
add_subdirectory(doc/interactivity)
add_subdirectory(doc/kinematics)
add_subdirectory(doc/motion_planning_api)
add_subdirectory(doc/motion_planning_pipeline)
add_subdirectory(doc/move_group_interface)
add_subdirectory(doc/move_group_python_interface)
add_subdirectory(doc/perception_pipeline)
add_subdirectory(doc/pick_place)
add_subdirectory(doc/planning)
add_subdirectory(doc/planning_scene)
add_subdirectory(doc/planning_scene_ros_api)
add_subdirectory(doc/robot_model_and_robot_state)
add_subdirectory(doc/state_display)
add_subdirectory(doc/subframes)
add_subdirectory(doc/tests)
add_subdirectory(doc/trajopt_planner)
add_subdirectory(doc/creating_moveit_plugins/lerp_motion_planner)
add_subdirectory(doc/moveit_cpp)
add_subdirectory(doc/collision_environments)
add_subdirectory(doc/visualizing_collisions)
add_subdirectory(doc/bullet_collision_checker)
add_subdirectory(doc/mesh_filter)
find_package(shape_msgs REQUIRED)
# add_subdirectory(moveit_visual_tools)

# find_package(moveit_visual_tools REQUIRED)
add_executable(PicknPlace src/PicknPlace.cpp)
target_link_libraries(PicknPlace ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS PicknPlace
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
