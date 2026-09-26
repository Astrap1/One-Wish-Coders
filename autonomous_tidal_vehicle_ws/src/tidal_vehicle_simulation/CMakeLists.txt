cmake_minimum_required(VERSION 3.8)
project(tidal_vehicle_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ament is optional so the Gazebo plugin can also be built with plain CMake
# (e.g. against a source build of Gazebo) for headless physics tests.
find_package(ament_cmake QUIET)

# --- Gazebo Harmonic (gz-sim 8) --------------------------------------------
# With ROS 2 Jazzy + ros_gz, Gazebo comes from the *_vendor packages; with a
# standalone Gazebo Harmonic install the gz-* CMake configs are found directly.
foreach(v gz_cmake_vendor gz_sim_vendor gz_plugin_vendor gz_transport_vendor gz_msgs_vendor)
  find_package(${v} QUIET)
endforeach()
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Tide visual system used by Person 3's corridor environment. It needs a
# gz-sim 8 release with Model::SetStatic (the one ROS 2 Jazzy installs);
# -DTIDAL_BUILD_TIDE_VISUAL=OFF skips it on older gz-sim 8 builds.
option(TIDAL_BUILD_TIDE_VISUAL "Build Person 3's tide visual system" ON)
if(TIDAL_BUILD_TIDE_VISUAL)
  add_library(tide_visual_system SHARED plugins/tide_visual_system.cpp)
  target_include_directories(tide_visual_system PRIVATE plugins/include)
  target_link_libraries(tide_visual_system PRIVATE
    gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
  install(TARGETS tide_visual_system LIBRARY DESTINATION lib)
endif()

# tidal_vehicle_plugins: hover::AirCushion (model), hover::TerrainZones and
# hover::ScriptedCommands (world). Gazebo finds it through
# GZ_SIM_SYSTEM_PLUGIN_PATH (set by tidal_vehicle_bringup's launch file).
add_library(tidal_vehicle_plugins SHARED
  plugins/src/AirCushion.cc
  plugins/src/TerrainZones.cc
  plugins/src/ScriptedCommands.cc)
target_include_directories(tidal_vehicle_plugins PRIVATE plugins/include)
target_link_libraries(tidal_vehicle_plugins PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)

install(TARGETS tidal_vehicle_plugins LIBRARY DESTINATION lib)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/vehicle_mobility_node.py
  scripts/lidar_scan_node.py
  scripts/terrain_costmap_node.py
  scripts/tide_manager.py
  scripts/collision_test_teleop.py
  scripts/gen_test_worlds.py
  scripts/gen_test_worlds_v2.py
  scripts/gen_test_worlds_v3.py
  scripts/collision_monitor_node.py
  DESTINATION lib/${PROJECT_NAME})

if(ament_cmake_FOUND)
  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(vehicle_mobility_test test/test_vehicle_mobility.py)
    ament_add_pytest_test(lidar_scan_test test/test_lidar_scan.py)
    ament_add_pytest_test(collision_monitor_test test/test_collision_monitor.py)
    ament_add_pytest_test(tide_manager_test test/test_tide_manager.py)
  endif()

  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
  ament_package()
endif()
