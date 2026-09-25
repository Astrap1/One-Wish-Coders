// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "tidal_vehicle_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__struct.h"
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // tide_state
#include "rosidl_runtime_c/string_functions.h"  // tide_state
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tidal_vehicle_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _TerrainState__ros_msg_type = tidal_vehicle_interfaces__msg__TerrainState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
bool cdr_serialize_tidal_vehicle_interfaces__msg__TerrainState(
  const tidal_vehicle_interfaces__msg__TerrainState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: tide_state
  {
    const rosidl_runtime_c__String * str = &ros_message->tide_state;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: tide_risk
  {
    cdr << ros_message->tide_risk;
  }

  // Field name: water_level_m
  {
    cdr << ros_message->water_level_m;
  }

  // Field name: tide_rate_m_per_minute
  {
    cdr << ros_message->tide_rate_m_per_minute;
  }

  // Field name: seconds_until_corridor_unsafe
  {
    cdr << ros_message->seconds_until_corridor_unsafe;
  }

  // Field name: corridor_traversable
  {
    cdr << (ros_message->corridor_traversable ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
bool cdr_deserialize_tidal_vehicle_interfaces__msg__TerrainState(
  eprosima::fastcdr::Cdr & cdr,
  tidal_vehicle_interfaces__msg__TerrainState * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: tide_state
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->tide_state.data) {
      rosidl_runtime_c__String__init(&ros_message->tide_state);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->tide_state,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'tide_state'\n");
      return false;
    }
  }

  // Field name: tide_risk
  {
    cdr >> ros_message->tide_risk;
  }

  // Field name: water_level_m
  {
    cdr >> ros_message->water_level_m;
  }

  // Field name: tide_rate_m_per_minute
  {
    cdr >> ros_message->tide_rate_m_per_minute;
  }

  // Field name: seconds_until_corridor_unsafe
  {
    cdr >> ros_message->seconds_until_corridor_unsafe;
  }

  // Field name: corridor_traversable
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->corridor_traversable = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t get_serialized_size_tidal_vehicle_interfaces__msg__TerrainState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TerrainState__ros_msg_type * ros_message = static_cast<const _TerrainState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: tide_state
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->tide_state.size + 1);

  // Field name: tide_risk
  {
    size_t item_size = sizeof(ros_message->tide_risk);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: water_level_m
  {
    size_t item_size = sizeof(ros_message->water_level_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: tide_rate_m_per_minute
  {
    size_t item_size = sizeof(ros_message->tide_rate_m_per_minute);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: seconds_until_corridor_unsafe
  {
    size_t item_size = sizeof(ros_message->seconds_until_corridor_unsafe);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: corridor_traversable
  {
    size_t item_size = sizeof(ros_message->corridor_traversable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t max_serialized_size_tidal_vehicle_interfaces__msg__TerrainState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: tide_state
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: tide_risk
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: water_level_m
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: tide_rate_m_per_minute
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: seconds_until_corridor_unsafe
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: corridor_traversable
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = tidal_vehicle_interfaces__msg__TerrainState;
    is_plain =
      (
      offsetof(DataType, corridor_traversable) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
bool cdr_serialize_key_tidal_vehicle_interfaces__msg__TerrainState(
  const tidal_vehicle_interfaces__msg__TerrainState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: tide_state
  {
    const rosidl_runtime_c__String * str = &ros_message->tide_state;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: tide_risk
  {
    cdr << ros_message->tide_risk;
  }

  // Field name: water_level_m
  {
    cdr << ros_message->water_level_m;
  }

  // Field name: tide_rate_m_per_minute
  {
    cdr << ros_message->tide_rate_m_per_minute;
  }

  // Field name: seconds_until_corridor_unsafe
  {
    cdr << ros_message->seconds_until_corridor_unsafe;
  }

  // Field name: corridor_traversable
  {
    cdr << (ros_message->corridor_traversable ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t get_serialized_size_key_tidal_vehicle_interfaces__msg__TerrainState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TerrainState__ros_msg_type * ros_message = static_cast<const _TerrainState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: tide_state
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->tide_state.size + 1);

  // Field name: tide_risk
  {
    size_t item_size = sizeof(ros_message->tide_risk);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: water_level_m
  {
    size_t item_size = sizeof(ros_message->water_level_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: tide_rate_m_per_minute
  {
    size_t item_size = sizeof(ros_message->tide_rate_m_per_minute);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: seconds_until_corridor_unsafe
  {
    size_t item_size = sizeof(ros_message->seconds_until_corridor_unsafe);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: corridor_traversable
  {
    size_t item_size = sizeof(ros_message->corridor_traversable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tidal_vehicle_interfaces
size_t max_serialized_size_key_tidal_vehicle_interfaces__msg__TerrainState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: tide_state
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: tide_risk
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: water_level_m
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: tide_rate_m_per_minute
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: seconds_until_corridor_unsafe
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: corridor_traversable
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = tidal_vehicle_interfaces__msg__TerrainState;
    is_plain =
      (
      offsetof(DataType, corridor_traversable) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _TerrainState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const tidal_vehicle_interfaces__msg__TerrainState * ros_message = static_cast<const tidal_vehicle_interfaces__msg__TerrainState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_tidal_vehicle_interfaces__msg__TerrainState(ros_message, cdr);
}

static bool _TerrainState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  tidal_vehicle_interfaces__msg__TerrainState * ros_message = static_cast<tidal_vehicle_interfaces__msg__TerrainState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_tidal_vehicle_interfaces__msg__TerrainState(cdr, ros_message);
}

static uint32_t _TerrainState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_tidal_vehicle_interfaces__msg__TerrainState(
      untyped_ros_message, 0));
}

static size_t _TerrainState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_tidal_vehicle_interfaces__msg__TerrainState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TerrainState = {
  "tidal_vehicle_interfaces::msg",
  "TerrainState",
  _TerrainState__cdr_serialize,
  _TerrainState__cdr_deserialize,
  _TerrainState__get_serialized_size,
  _TerrainState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TerrainState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TerrainState,
  get_message_typesupport_handle_function,
  &tidal_vehicle_interfaces__msg__TerrainState__get_type_hash,
  &tidal_vehicle_interfaces__msg__TerrainState__get_type_description,
  &tidal_vehicle_interfaces__msg__TerrainState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tidal_vehicle_interfaces, msg, TerrainState)() {
  return &_TerrainState__type_support;
}

#if defined(__cplusplus)
}
#endif
