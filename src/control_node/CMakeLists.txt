cmake_minimum_required(VERSION 3.0.2)
project(control_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  # simple_navigation_goals
  std_msgs
  message_generation
  actionlib
  move_base_msgs
  roscpp
  tf
)

# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CoordInfo.msg
  MovingInPolar.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ControlService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)


catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES control_node
 CATKIN_DEPENDS rospy roscpp std_msgs turtlebot3_move message_runtime #simple_navigation_goals 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/button.py
  src/control.py
  src/camera.py
  src/camera_try.py
  src/receive_coord_info_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(robot_control_node src/simple_navigation_goals.cpp)
target_link_libraries(robot_control_node ${catkin_LIBRARIES})
add_dependencies(robot_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})