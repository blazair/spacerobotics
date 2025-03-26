// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverVelocityStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_velocity_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverVelocityStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfc, 0x9d, 0x8e, 0xf4, 0x19, 0x06, 0x4d, 0x7b,
      0x35, 0x57, 0x77, 0x59, 0xc8, 0x94, 0x18, 0x43,
      0x3b, 0x96, 0x87, 0xb5, 0x36, 0x6f, 0x6d, 0xe0,
      0x80, 0xe7, 0x02, 0x7c, 0x38, 0xfc, 0x3d, 0x73,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverVelocityStatus__TYPE_NAME[] = "px4_msgs/msg/RoverVelocityStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__measured_speed_body_x[] = "measured_speed_body_x";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__speed_body_x_setpoint[] = "speed_body_x_setpoint";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__adjusted_speed_body_x_setpoint[] = "adjusted_speed_body_x_setpoint";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__measured_speed_body_y[] = "measured_speed_body_y";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__speed_body_y_setpoint[] = "speed_body_y_setpoint";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__adjusted_speed_body_y_setpoint[] = "adjusted_speed_body_y_setpoint";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__pid_throttle_body_x_integral[] = "pid_throttle_body_x_integral";
static char px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__pid_throttle_body_y_integral[] = "pid_throttle_body_y_integral";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverVelocityStatus__FIELDS[] = {
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__measured_speed_body_x, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__speed_body_x_setpoint, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__adjusted_speed_body_x_setpoint, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__measured_speed_body_y, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__speed_body_y_setpoint, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__adjusted_speed_body_y_setpoint, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__pid_throttle_body_x_integral, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverVelocityStatus__FIELD_NAME__pid_throttle_body_y_integral, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__RoverVelocityStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverVelocityStatus__TYPE_NAME, 32, 32},
      {px4_msgs__msg__RoverVelocityStatus__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 measured_speed_body_x          # [m/s] Measured speed in body x direction. Positiv: forwards, Negativ: backwards\n"
  "float32 speed_body_x_setpoint          # [m/s] Speed setpoint in body x direction. Positiv: forwards, Negativ: backwards\n"
  "float32 adjusted_speed_body_x_setpoint # [m/s] Post slew rate speed setpoint in body x direction. Positiv: forwards, Negativ: backwards\n"
  "float32 measured_speed_body_y          # [m/s] Measured speed in body y direction. Positiv: right, Negativ: left\n"
  "float32 speed_body_y_setpoint          # [m/s] Speed setpoint in body y direction. Positiv: right, Negativ: left (Only for mecanum rovers)\n"
  "float32 adjusted_speed_body_y_setpoint # [m/s] Post slew rate speed setpoint in body y direction. Positiv: right, Negativ: left (Only for mecanum rovers)\n"
  "float32 pid_throttle_body_x_integral   # Integral of the PID for the closed loop controller of the speed in body x direction\n"
  "float32 pid_throttle_body_y_integral   # Integral of the PID for the closed loop controller of the speed in body y direction\n"
  "\n"
  "# TOPICS rover_velocity_status";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverVelocityStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverVelocityStatus__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1125, 1125},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverVelocityStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverVelocityStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
