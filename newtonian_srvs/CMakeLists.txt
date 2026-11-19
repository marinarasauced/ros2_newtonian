cmake_minimum_required(VERSION 3.20)
project(newtonian_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Configure.srv"
  DEPENDENCIES
)

ament_package()
