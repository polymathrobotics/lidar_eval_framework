cmake_minimum_required(VERSION 3.8)
project(lidar_test_bench_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point4D.msg"
  "msg/Plane3D.msg"
  "msg/NumericalPointCloud.msg"
  "msg/ExpectedZone.msg"
  "msg/Visualization.msg"
  "srv/FilterCloud.srv"
  "srv/GetProfiles.srv"
  "srv/Visualization.srv"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
