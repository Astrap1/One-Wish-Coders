cmake_minimum_required(VERSION 3.8)
project(tidal_vehicle_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes models rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/gen_description.py scripts/gen_description_v2.py
  scripts/gen_description_v3.py DESTINATION lib/${PROJECT_NAME})

# Lets Gazebo resolve model://hovercraft after `source install/setup.bash`
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
