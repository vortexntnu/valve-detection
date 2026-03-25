cmake_minimum_required(VERSION 3.8)
project(valve_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/depth_image_processing.cpp
  src/pose_estimator.cpp
  src/ros_utils.cpp
  src/valve_pose_ros.cpp
)

target_link_libraries(${LIB_NAME} PUBLIC
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  message_filters
  sensor_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
  pcl_conversions
  vortex_msgs
  vortex_utils
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "valve_detection::ValvePoseNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
