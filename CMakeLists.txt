cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(radar_to_rviz)

#find_package (Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  visualization_msgs
  message_generation
  tf2
  tf2_ros
)

add_message_files(
  FILES
  TestPacket.msg
  CarData.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

#include_directories(
#  ${Eigen_INCLUDE_DIRS}
#  )
 
add_executable(radar_to_rviz_car_demo_node src/radar_to_rviz_car_demo.cpp src/CarDemoRadarNode.cpp)
add_dependencies(radar_to_rviz_car_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_to_rviz_car_demo_node ${catkin_LIBRARIES})

add_executable(tipi_to_rviz_node src/tipi_to_rviz.cpp)
add_dependencies(tipi_to_rviz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tipi_to_rviz_node ${catkin_LIBRARIES})

add_executable(tipi_to_rviz_points_node src/tipi_to_rviz_points.cpp)
add_dependencies(tipi_to_rviz_points_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tipi_to_rviz_points_node ${catkin_LIBRARIES})

add_executable(kanza_to_rviz_node src/kanza_to_rviz.cpp)
add_dependencies(kanza_to_rviz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kanza_to_rviz_node ${catkin_LIBRARIES})

add_executable(k79_to_rviz_node src/k79_to_rviz.cpp src/ClientK79.cpp)
add_dependencies(k79_to_rviz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(k79_to_rviz_node ${catkin_LIBRARIES})

add_executable(base_tf_broadcaster_node src/base_tf_broadcaster.cpp src/CarPoseEstimator.cpp)
add_dependencies(base_tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_tf_broadcaster_node ${catkin_LIBRARIES})

install(TARGETS radar_to_rviz_car_demo_node tipi_to_rviz_node kanza_to_rviz_node tipi_to_rviz_points_node k79_to_rviz_node base_tf_broadcaster_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
