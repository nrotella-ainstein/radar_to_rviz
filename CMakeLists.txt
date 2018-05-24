cmake_minimum_required(VERSION 2.8.3)
project(radar_to_rviz)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  visualization_msgs
)


catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(tipi_to_rviz_node src/tipi_to_rviz.cpp)
add_dependencies(tipi_to_rviz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tipi_to_rviz_node ${catkin_LIBRARIES})

add_executable(kanza_to_rviz_node src/kanza_to_rviz.cpp)
add_dependencies(kanza_to_rviz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kanza_to_rviz_node ${catkin_LIBRARIES})

 install(TARGETS tipi_to_rviz_node kanza_to_rviz_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

