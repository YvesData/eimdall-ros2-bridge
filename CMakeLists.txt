cmake_minimum_required(VERSION 3.8)
project(eimdall_ros2_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EimdallAnomaly.msg"
  "msg/EimdallHealth.msg"
  "msg/EimdallSensorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

ament_python_install_package(${PROJECT_NAME})

install(FILES package.xml README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/health_bridge
  scripts/anomaly_bridge
  scripts/ingest_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
