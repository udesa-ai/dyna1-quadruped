cmake_minimum_required(VERSION 3.8)
project(controler_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joint_msgs REQUIRED)
find_package(teleop_msgs REQUIRED)
find_package(custom_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


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


add_executable(interface src/dyna_real_interface.cpp src/algebra.cpp src/bezier.cpp src/leg_kinematics.cpp src/quad_kinematics.cpp src/trajectories.cpp)
ament_target_dependencies(interface rclcpp joint_msgs teleop_msgs std_msgs custom_sensor_msgs)

install(TARGETS
  interface
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
