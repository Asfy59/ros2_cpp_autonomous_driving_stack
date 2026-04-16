cmake_minimum_required(VERSION 3.8)
project(fusion_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auto_stack_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(fusion_core src/fusion_core.cpp)
add_executable(tracking_based_fusion src/tracking_based_fusion.cpp)
ament_target_dependencies(
  fusion_core
  auto_stack_msgs
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs)

ament_target_dependencies(
  tracking_based_fusion
  auto_stack_msgs
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs)

target_include_directories(fusion_core PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(tracking_based_fusion PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(fusion_core ${OpenCV_LIBS})
target_link_libraries(tracking_based_fusion ${OpenCV_LIBS})

install(TARGETS
  fusion_core
  tracking_based_fusion
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
