// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tidal_vehicle_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/safety_status.h"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_H_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_H_

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
// Member 'state'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SafetyStatus in the package tidal_vehicle_interfaces.
typedef struct tidal_vehicle_interfaces__msg__SafetyStatus
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String state;
  rosidl_runtime_c__String reason;
  bool return_required;
} tidal_vehicle_interfaces__msg__SafetyStatus;

// Struct for a sequence of tidal_vehicle_interfaces__msg__SafetyStatus.
typedef struct tidal_vehicle_interfaces__msg__SafetyStatus__Sequence
{
  tidal_vehicle_interfaces__msg__SafetyStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tidal_vehicle_interfaces__msg__SafetyStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__STRUCT_H_
