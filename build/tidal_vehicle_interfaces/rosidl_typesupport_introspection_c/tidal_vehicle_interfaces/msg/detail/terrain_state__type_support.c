// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__rosidl_typesupport_introspection_c.h"
#include "tidal_vehicle_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__functions.h"
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `tide_state`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tidal_vehicle_interfaces__msg__TerrainState__init(message_memory);
}

void tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_fini_function(void * message_memory)
{
  tidal_vehicle_interfaces__msg__TerrainState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tide_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, tide_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tide_risk",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, tide_risk),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "water_level_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, water_level_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tide_rate_m_per_minute",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, tide_rate_m_per_minute),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "seconds_until_corridor_unsafe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, seconds_until_corridor_unsafe),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "corridor_traversable",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tidal_vehicle_interfaces__msg__TerrainState, corridor_traversable),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_members = {
  "tidal_vehicle_interfaces__msg",  // message namespace
  "TerrainState",  // message name
  7,  // number of fields
  sizeof(tidal_vehicle_interfaces__msg__TerrainState),
  false,  // has_any_key_member_
  tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_member_array,  // message members
  tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_init_function,  // function to initialize message memory (memory has to be allocated)
  tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_type_support_handle = {
  0,
  &tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_members,
  get_message_typesupport_handle_function,
  &tidal_vehicle_interfaces__msg__TerrainState__get_type_hash,
  &tidal_vehicle_interfaces__msg__TerrainState__get_type_description,
  &tidal_vehicle_interfaces__msg__TerrainState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tidal_vehicle_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tidal_vehicle_interfaces, msg, TerrainState)() {
  tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_type_support_handle.typesupport_identifier) {
    tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tidal_vehicle_interfaces__msg__TerrainState__rosidl_typesupport_introspection_c__TerrainState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
