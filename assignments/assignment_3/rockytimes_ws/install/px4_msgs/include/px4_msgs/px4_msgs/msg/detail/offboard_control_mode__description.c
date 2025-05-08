// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OffboardControlMode.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/offboard_control_mode__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OffboardControlMode__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa7, 0x17, 0x67, 0x01, 0x56, 0x81, 0xb0, 0x0b,
      0x6f, 0x6e, 0xc9, 0x15, 0x02, 0x73, 0x0a, 0xe3,
      0x91, 0x03, 0x7b, 0x9d, 0x93, 0xf2, 0xc3, 0x75,
      0xb9, 0x74, 0xe2, 0xbb, 0xb5, 0xce, 0x16, 0x42,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OffboardControlMode__TYPE_NAME[] = "px4_msgs/msg/OffboardControlMode";

// Define type names, field names, and default values
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__velocity[] = "velocity";
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__acceleration[] = "acceleration";
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__attitude[] = "attitude";
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__body_rate[] = "body_rate";
static char px4_msgs__msg__OffboardControlMode__FIELD_NAME__actuator[] = "actuator";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OffboardControlMode__FIELDS[] = {
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__attitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__body_rate, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OffboardControlMode__FIELD_NAME__actuator, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__OffboardControlMode__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OffboardControlMode__TYPE_NAME, 32, 32},
      {px4_msgs__msg__OffboardControlMode__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Off-board control mode\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "bool position\n"
  "bool velocity\n"
  "bool acceleration\n"
  "bool attitude\n"
  "bool body_rate\n"
  "bool actuator";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OffboardControlMode__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OffboardControlMode__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 175, 175},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OffboardControlMode__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OffboardControlMode__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
