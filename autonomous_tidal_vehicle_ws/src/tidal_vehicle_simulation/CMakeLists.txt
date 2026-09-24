cmake_minimum_required(VERSION 3.8)
project(tidal_vehicle_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models plugins worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
