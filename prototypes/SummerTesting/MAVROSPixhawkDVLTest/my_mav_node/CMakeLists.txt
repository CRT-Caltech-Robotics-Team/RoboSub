cmake_minimum_required(VERSION 3.8)
project(my_mav_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(myprogram src/myprogram.cpp)
target_include_directories(myprogram PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(myprogram PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  myprogram
  "rclcpp"
  "mavros_msgs"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)

target_link_libraries(
  myprogram
  Eigen3::Eigen
)

add_executable(offboard_node src/offboard_node.cpp)
target_compile_features(offboard_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  offboard_node
  "rclcpp"
  "mavros_msgs"
)

add_executable(status_node src/status_node.cpp)
target_compile_features(status_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  status_node
  "rclcpp"
  "mavros_msgs"
  "sensor_msgs"
  "geometry_msgs"
)

add_executable(thruster_check src/thruster_check.cpp)
target_compile_features(thruster_check PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  thruster_check
  "rclcpp"
  "mavros_msgs"
)

install(TARGETS myprogram offboard_node status_node thruster_check
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

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
