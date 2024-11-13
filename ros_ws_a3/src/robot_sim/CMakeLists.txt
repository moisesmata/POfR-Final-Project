cmake_minimum_required(VERSION 3.8)
project(robot_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# find_package(baxter_core_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  include
)

add_library(robot_sim SHARED
  src/joint_state_publisher_.cpp
  src/robot.cpp
  src/velocity_controller.cpp
  src/position_controller.cpp
  src/trajectory_executer.cpp
)
ament_target_dependencies(robot_sim
  rclcpp
  sensor_msgs
  trajectory_msgs
  # baxter_core_msgs
  urdf
)

add_executable(robot_sim_bringup src/robot_sim_bringup.cpp)
ament_target_dependencies(robot_sim_bringup
  rclcpp
  sensor_msgs
  trajectory_msgs
  # baxter_core_msgs
  urdf
)
target_link_libraries(robot_sim_bringup robot_sim)

install(TARGETS
  robot_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  robot_sim_bringup
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  robot_sim/position_command.py
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
