// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__rosidl_typesupport_introspection_c.h"
#include "tidal_vehicle_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__functions.h"
#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `fault`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tidal_vehicle_interfaces__msg__VehicleHealth__init(message_memory);
}

void tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_fini_function(void * message_memory)
{
  tidal_vehicle_interfaces__msg__VehicleHealth__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, battery_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "return_reserve_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, return_reserve_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mobility_health_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, mobility_health_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "link_ok",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, link_ok),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "payload_secured",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, payload_secured),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fault",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__VehicleHealth, fault),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_members = {
  "tidal_vehicle_interfaces__msg",  // message namespace
  "VehicleHealth",  // message name
  7,  // number of fields
  sizeof(tidal_vehicle_interfaces__msg__VehicleHealth),
  false,  // has_any_key_member_
  tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_member_array,  // message members
  tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_init_function,  // function to initialize message memory (memory has to be allocated)
  tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_type_support_handle = {
  0,
  &tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_members,
  get_message_typesupport_handle_function,
  &tidal_vehicle_interfaces__msg__VehicleHealth__get_type_hash,
  &tidal_vehicle_interfaces__msg__VehicleHealth__get_type_description,
  &tidal_vehicle_interfaces__msg__VehicleHealth__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tidal_vehicle_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tidal_vehicle_interfaces, msg, VehicleHealth)() {
  tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_type_support_handle.typesupport_identifier) {
    tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tidal_vehicle_interfaces__msg__VehicleHealth__rosidl_typesupport_introspection_c__VehicleHealth_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
