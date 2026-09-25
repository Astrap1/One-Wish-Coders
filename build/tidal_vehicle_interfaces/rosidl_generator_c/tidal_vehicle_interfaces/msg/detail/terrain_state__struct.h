// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/terrain_state.h"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__STRUCT_H_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'tide_state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TerrainState in the package tidal_vehicle_interfaces.
typedef struct tidal_vehicle_interfaces__msg__TerrainState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String tide_state;
  float tide_risk;
  float water_level_m;
  float tide_rate_m_per_minute;
  float seconds_until_corridor_unsafe;
  bool corridor_traversable;
} tidal_vehicle_interfaces__msg__TerrainState;

// Struct for a sequence of tidal_vehicle_interfaces__msg__TerrainState.
typedef struct tidal_vehicle_interfaces__msg__TerrainState__Sequence
{
  tidal_vehicle_interfaces__msg__TerrainState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tidal_vehicle_interfaces__msg__TerrainState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__STRUCT_H_
