cmake_minimum_required(VERSION 3.5)
project(pharma_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add hardware interface library (commented out for Python motor driver)
# add_library(rpi_hardware_interface SHARED
#   src/rpi_hardware_interface.cpp
# )

# target_include_directories(rpi_hardware_interface PRIVATE
#   include
# )

# ament_target_dependencies(rpi_hardware_interface
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
# )

# Export plugin (commented out for Python motor driver)
# pluginlib_export_plugin_description_file(hardware_interface rpi_hardware_interface.xml)

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

# Install targets (commented out for Python motor driver)
# install(
#   TARGETS rpi_hardware_interface
#   DESTINATION lib
# )

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/rpi_motor_driver.py
  scripts/motor_encoder_test.py
  scripts/interactive_motor_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
