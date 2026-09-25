// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice
#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tidal_vehicle_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
bool cdr_serialize_tidal_vehicle_interfaces__msg__TerrainState(
  const tidal_vehicle_interfaces__msg__TerrainState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
bool cdr_deserialize_tidal_vehicle_interfaces__msg__TerrainState(
  eprosima::fastcdr::Cdr &,
  tidal_vehicle_interfaces__msg__TerrainState * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t get_serialized_size_tidal_vehicle_interfaces__msg__TerrainState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t max_serialized_size_tidal_vehicle_interfaces__msg__TerrainState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
bool cdr_serialize_key_tidal_vehicle_interfaces__msg__TerrainState(
  const tidal_vehicle_interfaces__msg__TerrainState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t get_serialized_size_key_tidal_vehicle_interfaces__msg__TerrainState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t max_serialized_size_key_tidal_vehicle_interfaces__msg__TerrainState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tidal_vehicle_interfaces, msg, TerrainState)();

#ifdef __cplusplus
}
#endif

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
