cmake_minimum_required(VERSION 2.8.3)
project(mtg_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  mtg_messages
  mtg_task_allocation
  message_runtime
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs geometry_msgs mtg_messages mtg_task_allocation tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mtg_planner src/cbs_node.cpp)
add_executable(mtg_planner_tester src/planner_tester.cpp)
add_dependencies(mtg_planner ${catkin_EXPORTED_TARGETS})
add_dependencies(mtg_planner_tester ${catkin_EXPORTED_TARGETS})
target_link_libraries(mtg_planner ${catkin_LIBRARIES} )
target_link_libraries(mtg_planner_tester ${catkin_LIBRARIES})
