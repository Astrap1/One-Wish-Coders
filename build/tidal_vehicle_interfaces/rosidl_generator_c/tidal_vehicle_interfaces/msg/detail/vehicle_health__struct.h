// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/vehicle_health.h"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__STRUCT_H_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__STRUCT_H_

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
// Member 'fault'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/VehicleHealth in the package tidal_vehicle_interfaces.
typedef struct tidal_vehicle_interfaces__msg__VehicleHealth
{
  std_msgs__msg__Header header;
  float battery_percent;
  float return_reserve_percent;
  float mobility_health_percent;
  bool link_ok;
  bool payload_secured;
  rosidl_runtime_c__String fault;
} tidal_vehicle_interfaces__msg__VehicleHealth;

// Struct for a sequence of tidal_vehicle_interfaces__msg__VehicleHealth.
typedef struct tidal_vehicle_interfaces__msg__VehicleHealth__Sequence
{
  tidal_vehicle_interfaces__msg__VehicleHealth * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tidal_vehicle_interfaces__msg__VehicleHealth__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__STRUCT_H_
