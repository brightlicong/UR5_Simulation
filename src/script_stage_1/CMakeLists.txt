cmake_minimum_required(VERSION 3.0.2)
project(script_stage_1)

set(PCL_DIR "/usr/share/pcl-1.13/")

find_package(catkin REQUIRED COMPONENTS
  PCL
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(OBB src/OBB.cpp)
add_library(DBSCAN src/DBSCAN.cpp )

add_dependencies(OBB ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(DBSCAN ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(OBB
   ${catkin_LIBRARIES}
)

target_link_libraries(DBSCAN
   ${catkin_LIBRARIES}
)

add_executable(main 
  src/main.cpp
  src/OBB.cpp
  src/DBSCAN.cpp)


add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main
  ${catkin_LIBRARIES}
)

