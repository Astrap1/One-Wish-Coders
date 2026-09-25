# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target tidal_vehicle_interfaces::tidal_vehicle_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${tidal_vehicle_interfaces_TARGETS}.
if(tidal_vehicle_interfaces_TARGETS AND NOT TARGET tidal_vehicle_interfaces::tidal_vehicle_interfaces)
  add_library(tidal_vehicle_interfaces::tidal_vehicle_interfaces INTERFACE IMPORTED)
  set_target_properties(tidal_vehicle_interfaces::tidal_vehicle_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${tidal_vehicle_interfaces_TARGETS}")
endif()
