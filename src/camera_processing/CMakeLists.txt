cmake_minimum_required(VERSION 3.8)
project(camera_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(auto_stack_msgs REQUIRED)
# find_package(onnxruntime REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(ONNXRUNTIME_ROOT "/usr/local/onnxruntime")

add_executable(camera_processing src/camera_processing.cpp src/yolo_detector.cpp)
ament_target_dependencies(camera_processing rclcpp sensor_msgs cv_bridge vision_msgs auto_stack_msgs OpenCV)

target_include_directories(camera_processing PUBLIC include ${OpenCV_INCLUDE_DIRS} ${ONNXRUNTIME_ROOT}/include)

target_link_directories(camera_processing
  PUBLIC ${OpenCV_LIBRARY_DIRS} ${ONNXRUNTIME_ROOT}/lib)

target_link_libraries(camera_processing
  ${OpenCV_LIBS}
  onnxruntime)

install(TARGETS
  camera_processing
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
