cmake_minimum_required(VERSION 3.8)
project(mega_diff_drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(diff_drive_controller REQUIRED)

# Manual path for libserial
set(LibSerial_INCLUDE_DIRS "/usr/local/include")
set(LibSerial_LIBRARIES "/usr/local/lib/libserial.so")

add_library(mega_diff_drive_hardware SHARED
  src/mega_diff_drive_hardware.cpp
)

# target libserial
target_include_directories(mega_diff_drive_hardware PUBLIC
  ${LibSerial_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mega_diff_drive_hardware
  ${LibSerial_LIBRARIES}
)

target_include_directories(mega_diff_drive_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)    

# install header file for other packages that might use it
install(DIRECTORY include/
  DESTINATION include/
)

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface mega_diff_drive_hardware.xml)

# install the library
install(TARGETS mega_diff_drive_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_target_dependencies(mega_diff_drive_hardware
  rclcpp
  hardware_interface
  controller_interface
  pluginlib
  )

target_include_directories(mega_diff_drive_hardware PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
