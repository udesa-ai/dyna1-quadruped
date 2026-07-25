cmake_minimum_required(VERSION 3.8)
project(opti_vel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



add_executable(velocity_publisher src/velocity_publisher.cpp)
add_executable(projected_gravity_publisher src/projected_gravity_publisher.cpp)

ament_target_dependencies(velocity_publisher rclcpp std_msgs geometry_msgs mocap4r2_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(projected_gravity_publisher rclcpp std_msgs geometry_msgs mocap4r2_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  velocity_publisher
  projected_gravity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
