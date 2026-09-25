// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice

#include "tidal_vehicle_interfaces/msg/detail/terrain_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tidal_vehicle_interfaces
const rosidl_type_hash_t *
tidal_vehicle_interfaces__msg__TerrainState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x50, 0x3a, 0xb2, 0xfc, 0xf6, 0x08, 0xca, 0xdd,
      0x28, 0xb4, 0xef, 0xf5, 0x82, 0x3f, 0xdc, 0x34,
      0xb4, 0xea, 0x88, 0x64, 0x52, 0x80, 0x8b, 0xc3,
      0xc7, 0xbf, 0xed, 0x8b, 0xb7, 0x3d, 0x4c, 0x62,
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

static char tidal_vehicle_interfaces__msg__TerrainState__TYPE_NAME[] = "tidal_vehicle_interfaces/msg/TerrainState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__header[] = "header";
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__tide_state[] = "tide_state";
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__tide_risk[] = "tide_risk";
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__water_level_m[] = "water_level_m";
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__tide_rate_m_per_minute[] = "tide_rate_m_per_minute";
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__seconds_until_corridor_unsafe[] = "seconds_until_corridor_unsafe";
static char tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__corridor_traversable[] = "corridor_traversable";

static rosidl_runtime_c__type_description__Field tidal_vehicle_interfaces__msg__TerrainState__FIELDS[] = {
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__tide_state, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__tide_risk, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__water_level_m, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__tide_rate_m_per_minute, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__seconds_until_corridor_unsafe, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tidal_vehicle_interfaces__msg__TerrainState__FIELD_NAME__corridor_traversable, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription tidal_vehicle_interfaces__msg__TerrainState__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
tidal_vehicle_interfaces__msg__TerrainState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tidal_vehicle_interfaces__msg__TerrainState__TYPE_NAME, 41, 41},
      {tidal_vehicle_interfaces__msg__TerrainState__FIELDS, 7, 7},
    },
    {tidal_vehicle_interfaces__msg__TerrainState__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "string tide_state\n"
  "float32 tide_risk\n"
  "float32 water_level_m\n"
  "float32 tide_rate_m_per_minute\n"
  "float32 seconds_until_corridor_unsafe\n"
  "bool corridor_traversable";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tidal_vehicle_interfaces__msg__TerrainState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tidal_vehicle_interfaces__msg__TerrainState__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 176, 176},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tidal_vehicle_interfaces__msg__TerrainState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tidal_vehicle_interfaces__msg__TerrainState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
