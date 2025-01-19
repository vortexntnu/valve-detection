cmake_minimum_required(VERSION 3.8)
project(valve_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # Correct the OpenCV package name here
find_package(PCL REQUIRED)  # Point Cloud Library'
find_package(pcl_conversions REQUIRED)

# Add executable
add_executable(valve_detection src/valve_detection.cpp)

# Include OpenCV and PCL directories
include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Link libraries
ament_target_dependencies(valve_detection
  rclcpp
  cv_bridge
  sensor_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  PCL
  pcl_conversions
)

# Link OpenCV and PCL to your target
target_link_libraries(valve_detection 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}  # Ensure that PCL libraries are linked
)

# Install
install(TARGETS
  valve_detection
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
