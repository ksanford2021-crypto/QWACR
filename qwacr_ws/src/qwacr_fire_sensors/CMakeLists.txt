cmake_minimum_required(VERSION 3.8)
project(qwacr_fire_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY qwacr_fire_sensors
  DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
)

install(
  PROGRAMS
    scripts/sen54_node.py
    scripts/mlx90640_node.py
    scripts/dgs2_h2_node.py
    scripts/dgs2_co_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/dgs2_decode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
