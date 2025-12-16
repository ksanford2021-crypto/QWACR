cmake_minimum_required(VERSION 3.8)
project(qwacr_build)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# `launch_ros` is only needed at runtime for launching; some ROS2
# binary installations provide launch_ros as a Python package without a
# CMake config. If you don't need build-time symbols from launch_ros,
# avoid calling find_package here to prevent CMake config errors.
# find_package(launch_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mega_diff_drive_control REQUIRED)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
  )
install(DIRECTORY urdf 
  DESTINATION share/${PROJECT_NAME}/
  )
install(PROGRAMS 
  scripts/serial_sim.py
  scripts/arduino_sim.py
  scripts/activate_controllers.py
  scripts/activate_controllers_custom_timeout.py
  DESTINATION lib/${PROJECT_NAME}
  )
install(FILES launch/qwacr_display.rviz
  DESTINATION share/${PROJECT_NAME}/
  ) 
# install the configuration directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
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
