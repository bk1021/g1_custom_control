cmake_minimum_required(VERSION 3.8)
project(g1_custom_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Add the executable
add_executable(g1_homing_node src/g1_homing_node.cpp src/motor_crc_hg.cpp)
target_include_directories(g1_homing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(g1_homing_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(g1_homing_node rclcpp unitree_hg)

add_executable(g1_moveit_bridge src/g1_moveit_bridge.cpp src/motor_crc_hg.cpp)
target_include_directories(g1_moveit_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(g1_moveit_bridge PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(g1_moveit_bridge rclcpp rclcpp_action sensor_msgs control_msgs unitree_hg)

add_executable(g1_ik_demo src/g1_ik_demo.cpp)
target_compile_features(g1_ik_demo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(g1_ik_demo 
  rclcpp 
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS 
  g1_homing_node
  g1_moveit_bridge
  g1_ik_demo
  DESTINATION lib/${PROJECT_NAME}
)

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