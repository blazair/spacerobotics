// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorGps.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_gps__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorGps__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x87, 0xdf, 0x2d, 0x73, 0xca, 0x9f, 0xbb, 0xd0,
      0x60, 0x7b, 0xdc, 0x31, 0x33, 0xc8, 0x44, 0x33,
      0xb0, 0x9c, 0x58, 0xc6, 0x35, 0x17, 0x8d, 0xcd,
      0x12, 0xc9, 0xb9, 0xfb, 0xff, 0x0d, 0xdc, 0xba,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorGps__TYPE_NAME[] = "px4_msgs/msg/SensorGps";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorGps__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorGps__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorGps__FIELD_NAME__lat[] = "lat";
static char px4_msgs__msg__SensorGps__FIELD_NAME__lon[] = "lon";
static char px4_msgs__msg__SensorGps__FIELD_NAME__alt[] = "alt";
static char px4_msgs__msg__SensorGps__FIELD_NAME__alt_ellipsoid[] = "alt_ellipsoid";
static char px4_msgs__msg__SensorGps__FIELD_NAME__s_variance_m_s[] = "s_variance_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__c_variance_rad[] = "c_variance_rad";
static char px4_msgs__msg__SensorGps__FIELD_NAME__fix_type[] = "fix_type";
static char px4_msgs__msg__SensorGps__FIELD_NAME__eph[] = "eph";
static char px4_msgs__msg__SensorGps__FIELD_NAME__epv[] = "epv";
static char px4_msgs__msg__SensorGps__FIELD_NAME__hdop[] = "hdop";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vdop[] = "vdop";
static char px4_msgs__msg__SensorGps__FIELD_NAME__noise_per_ms[] = "noise_per_ms";
static char px4_msgs__msg__SensorGps__FIELD_NAME__automatic_gain_control[] = "automatic_gain_control";
static char px4_msgs__msg__SensorGps__FIELD_NAME__jamming_state[] = "jamming_state";
static char px4_msgs__msg__SensorGps__FIELD_NAME__jamming_indicator[] = "jamming_indicator";
static char px4_msgs__msg__SensorGps__FIELD_NAME__spoofing_state[] = "spoofing_state";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_m_s[] = "vel_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_n_m_s[] = "vel_n_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_e_m_s[] = "vel_e_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_d_m_s[] = "vel_d_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__cog_rad[] = "cog_rad";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_ned_valid[] = "vel_ned_valid";
static char px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_time_relative[] = "timestamp_time_relative";
static char px4_msgs__msg__SensorGps__FIELD_NAME__time_utc_usec[] = "time_utc_usec";
static char px4_msgs__msg__SensorGps__FIELD_NAME__satellites_used[] = "satellites_used";
static char px4_msgs__msg__SensorGps__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__SensorGps__FIELD_NAME__heading_offset[] = "heading_offset";
static char px4_msgs__msg__SensorGps__FIELD_NAME__heading_accuracy[] = "heading_accuracy";
static char px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_injection_rate[] = "rtcm_injection_rate";
static char px4_msgs__msg__SensorGps__FIELD_NAME__selected_rtcm_instance[] = "selected_rtcm_instance";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorGps__FIELDS[] = {
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__lat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__lon, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__alt, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__alt_ellipsoid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__s_variance_m_s, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__c_variance_rad, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__fix_type, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__eph, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__epv, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__hdop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vdop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__noise_per_ms, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__automatic_gain_control, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__jamming_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__jamming_indicator, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__spoofing_state, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_m_s, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_n_m_s, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_e_m_s, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_d_m_s, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__cog_rad, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_ned_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_time_relative, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__time_utc_usec, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__satellites_used, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__heading_offset, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__heading_accuracy, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_injection_rate, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__selected_rtcm_instance, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorGps__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorGps__TYPE_NAME, 22, 22},
      {px4_msgs__msg__SensorGps__FIELDS, 33, 33},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GPS position in WGS84 coordinates.\n"
  "# the field 'timestamp' is for the position & velocity (microseconds)\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint64 timestamp_sample\n"
  "\n"
  "uint32 device_id                # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "int32 lat\\t\\t\\t# Latitude in 1E-7 degrees\n"
  "int32 lon\\t\\t\\t# Longitude in 1E-7 degrees\n"
  "int32 alt\\t\\t\\t# Altitude in 1E-3 meters above MSL, (millimetres)\n"
  "int32 alt_ellipsoid \\t\\t# Altitude in 1E-3 meters bove Ellipsoid, (millimetres)\n"
  "\n"
  "float32 s_variance_m_s\\t\\t# GPS speed accuracy estimate, (metres/sec)\n"
  "float32 c_variance_rad\\t\\t# GPS course accuracy estimate, (radians)\n"
  "uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.\n"
  "\n"
  "float32 eph\\t\\t\\t# GPS horizontal position accuracy (metres)\n"
  "float32 epv\\t\\t\\t# GPS vertical position accuracy (metres)\n"
  "\n"
  "float32 hdop\\t\\t\\t# Horizontal dilution of precision\n"
  "float32 vdop\\t\\t\\t# Vertical dilution of precision\n"
  "\n"
  "int32 noise_per_ms\\t\\t# GPS noise per millisecond\n"
  "uint16 automatic_gain_control   # Automatic gain control monitor\n"
  "\n"
  "uint8 JAMMING_STATE_UNKNOWN  = 0\n"
  "uint8 JAMMING_STATE_OK       = 1\n"
  "uint8 JAMMING_STATE_WARNING  = 2\n"
  "uint8 JAMMING_STATE_CRITICAL = 3\n"
  "uint8 jamming_state\\t\\t# indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical\n"
  "int32 jamming_indicator\\t\\t# indicates jamming is occurring\n"
  "\n"
  "uint8 SPOOFING_STATE_UNKNOWN   = 0\n"
  "uint8 SPOOFING_STATE_NONE      = 1\n"
  "uint8 SPOOFING_STATE_INDICATED = 2\n"
  "uint8 SPOOFING_STATE_MULTIPLE  = 3\n"
  "uint8 spoofing_state\\t\\t# indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical\n"
  "\n"
  "float32 vel_m_s\\t\\t\\t# GPS ground speed, (metres/sec)\n"
  "float32 vel_n_m_s\\t\\t# GPS North velocity, (metres/sec)\n"
  "float32 vel_e_m_s\\t\\t# GPS East velocity, (metres/sec)\n"
  "float32 vel_d_m_s\\t\\t# GPS Down velocity, (metres/sec)\n"
  "float32 cog_rad\\t\\t\\t# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)\n"
  "bool vel_ned_valid\\t\\t# True if NED velocity is valid\n"
  "\n"
  "int32 timestamp_time_relative\\t# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)\n"
  "uint64 time_utc_usec\\t\\t# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0\n"
  "\n"
  "uint8 satellites_used\\t\\t# Number of satellites used\n"
  "\n"
  "float32 heading\\t\\t\\t# heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])\n"
  "float32 heading_offset\\t\\t# heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])\n"
  "float32 heading_accuracy\\t# heading accuracy (rad, [0, 2PI])\n"
  "\n"
  "float32 rtcm_injection_rate\\t# RTCM message injection rate Hz\n"
  "uint8 selected_rtcm_instance\\t# uorb instance that is being used for RTCM corrections\n"
  "\n"
  "# TOPICS sensor_gps vehicle_gps_position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorGps__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorGps__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 3133, 3133},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorGps__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorGps__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
