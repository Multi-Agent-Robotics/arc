cmake_minimum_required(VERSION 3.5)
project(arc_base)

# If clang is installed on the system, use it
# as the error messages are much more readable than gcc's
if(EXISTS "/usr/bin/clang++")
  set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
  message(STATUS "Using clang++ compiler")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# include "include" directory
include_directories(include)

set(SHARED_AMENT_DEPS rclcpp arc_msgs vesc_msgs geometry_msgs std_msgs nav_msgs tf2_geometry_msgs tf2_ros tf2_eigen tf2_msgs tf2)
foreach(ament_deps ${SHARED_AMENT_DEPS})
    find_package(${ament_deps} REQUIRED)
endforeach()


# find_package(rclcpp REQUIRED)
# find_package(arc_msgs REQUIRED)
# find_package(vesc_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

add_executable(twist_to_diff_drive_node src/twist_to_diff_drive_node.cpp)
# ament_target_dependencies(twist_to_diff_drive_node arc_msgs vesc_msgs geometry_msgs)
ament_target_dependencies(twist_to_diff_drive_node ${SHARED_AMENT_DEPS})

add_executable(low_level_motor_controller src/low_level_motor_controller.cpp)
target_include_directories(low_level_motor_controller PRIVATE src)
ament_target_dependencies(low_level_motor_controller ${SHARED_AMENT_DEPS})
# ament_target_dependencies(low_level_motor_controller rclcpp arc_msgs vesc_msgs geometry_msgs)

add_executable(pos_controller src/pos_controller.cpp)
target_include_directories(pos_controller PRIVATE src)
ament_target_dependencies(pos_controller ${SHARED_AMENT_DEPS})

add_executable(motor_erpm_to_rad_per_sec_node src/motor_erpm_to_rad_per_sec_node.cpp)
target_include_directories(motor_erpm_to_rad_per_sec_node PRIVATE src)
ament_target_dependencies(motor_erpm_to_rad_per_sec_node ${SHARED_AMENT_DEPS})

install(TARGETS
  twist_to_diff_drive_node
  low_level_motor_controller
  pos_controller
  motor_erpm_to_rad_per_sec_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
