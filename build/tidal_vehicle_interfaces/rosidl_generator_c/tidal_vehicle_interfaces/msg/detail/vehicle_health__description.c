// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice

#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tidal_vehicle_interfaces
const rosidl_type_hash_t *
tidal_vehicle_interfaces__msg__VehicleHealth__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6a, 0xff, 0x32, 0x22, 0x37, 0xb6, 0xe3, 0x91,
      0x88, 0x2a, 0x99, 0x43, 0x40, 0xd3, 0xd4, 0xfe,
      0xc7, 0x12, 0x56, 0x9f, 0x2e, 0xc8, 0x97, 0x81,
      0x88, 0x82, 0xc3, 0x54, 0xf7, 0xb2, 0xb3, 0x96,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char tidal_vehicle_interfaces__msg__VehicleHealth__TYPE_NAME[] = "tidal_vehicle_interfaces/msg/VehicleHealth";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__header[] = "header";
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__battery_percent[] = "battery_percent";
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__return_reserve_percent[] = "return_reserve_percent";
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__mobility_health_percent[] = "mobility_health_percent";
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__link_ok[] = "link_ok";
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__payload_secured[] = "payload_secured";
static char tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__fault[] = "fault";

static rosidl_runtime_c__type_description__Field tidal_vehicle_interfaces__msg__VehicleHealth__FIELDS[] = {
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__battery_percent, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__return_reserve_percent, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__mobility_health_percent, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__link_ok, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__payload_secured, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__VehicleHealth__FIELD_NAME__fault, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription tidal_vehicle_interfaces__msg__VehicleHealth__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tidal_vehicle_interfaces__msg__VehicleHealth__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tidal_vehicle_interfaces__msg__VehicleHealth__TYPE_NAME, 42, 42},
      {tidal_vehicle_interfaces__msg__VehicleHealth__FIELDS, 7, 7},
    },
    {tidal_vehicle_interfaces__msg__VehicleHealth__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "float32 battery_percent\n"
  "float32 return_reserve_percent\n"
  "float32 mobility_health_percent\n"
  "bool link_ok\n"
  "bool payload_secured\n"
  "string fault";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tidal_vehicle_interfaces__msg__VehicleHealth__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tidal_vehicle_interfaces__msg__VehicleHealth__TYPE_NAME, 42, 42},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 157, 157},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tidal_vehicle_interfaces__msg__VehicleHealth__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tidal_vehicle_interfaces__msg__VehicleHealth__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
