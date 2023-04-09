cmake_minimum_required(VERSION 3.0.2)
project(P2P_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES script_stage_2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(script src/send_waypoints.cpp)

add_dependencies(script ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(script
  ${catkin_LIBRARIES}
)

#catkin_install_python( PROGRAMS scripts/send_waypoints.py 
#DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})