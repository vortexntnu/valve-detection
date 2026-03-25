cmake_minimum_required(VERSION 3.8)
project(valve_subtype_resolver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/valve_subtype_resolver.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  vortex_msgs
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "valve_subtype_resolver::ValveSubtypeResolverNode"
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
