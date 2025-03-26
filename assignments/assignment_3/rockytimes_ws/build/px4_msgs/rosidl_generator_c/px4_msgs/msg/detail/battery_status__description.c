// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/battery_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__BatteryStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6e, 0x22, 0x36, 0x62, 0xbe, 0x66, 0xb8, 0xbc,
      0x7f, 0x04, 0xa2, 0x10, 0xf1, 0xc3, 0x8c, 0x23,
      0x20, 0x60, 0x62, 0x12, 0x55, 0x88, 0xa6, 0xdd,
      0x74, 0x18, 0x9d, 0x49, 0x75, 0xd8, 0xb2, 0x8f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__BatteryStatus__TYPE_NAME[] = "px4_msgs/msg/BatteryStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__connected[] = "connected";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_v[] = "voltage_v";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__current_a[] = "current_a";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__current_average_a[] = "current_average_a";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__discharged_mah[] = "discharged_mah";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining[] = "remaining";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__scale[] = "scale";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__time_remaining_s[] = "time_remaining_s";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__temperature[] = "temperature";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__cell_count[] = "cell_count";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__source[] = "source";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__priority[] = "priority";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__capacity[] = "capacity";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__cycle_count[] = "cycle_count";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__average_time_to_empty[] = "average_time_to_empty";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__serial_number[] = "serial_number";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__manufacture_date[] = "manufacture_date";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__state_of_health[] = "state_of_health";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__max_error[] = "max_error";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__id[] = "id";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__interface_error[] = "interface_error";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_cell_v[] = "voltage_cell_v";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__max_cell_voltage_delta[] = "max_cell_voltage_delta";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__is_powering_off[] = "is_powering_off";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__is_required[] = "is_required";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__faults[] = "faults";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__warning[] = "warning";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__full_charge_capacity_wh[] = "full_charge_capacity_wh";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining_capacity_wh[] = "remaining_capacity_wh";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__over_discharge_count[] = "over_discharge_count";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__nominal_voltage[] = "nominal_voltage";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__internal_resistance_estimate[] = "internal_resistance_estimate";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate[] = "ocv_estimate";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate_filtered[] = "ocv_estimate_filtered";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__volt_based_soc_estimate[] = "volt_based_soc_estimate";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_prediction[] = "voltage_prediction";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__prediction_error[] = "prediction_error";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__estimation_covariance_norm[] = "estimation_covariance_norm";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__BatteryStatus__FIELDS[] = {
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__connected, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_v, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__current_a, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__current_average_a, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__discharged_mah, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__scale, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__time_remaining_s, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__cell_count, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__priority, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__capacity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__cycle_count, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__average_time_to_empty, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__serial_number, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__manufacture_date, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__state_of_health, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__max_error, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__interface_error, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_cell_v, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      14,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__max_cell_voltage_delta, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__is_powering_off, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__is_required, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__faults, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__warning, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__full_charge_capacity_wh, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining_capacity_wh, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__over_discharge_count, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__nominal_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__internal_resistance_estimate, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate_filtered, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__volt_based_soc_estimate, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_prediction, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__prediction_error, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__estimation_covariance_norm, 26, 26},
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
px4_msgs__msg__BatteryStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__BatteryStatus__TYPE_NAME, 26, 26},
      {px4_msgs__msg__BatteryStatus__FIELDS, 39, 39},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp\\t\\t\\t\\t# time since system start (microseconds)\n"
  "bool connected\\t\\t\\t\\t\\t# Whether or not a battery is connected, based on a voltage threshold\n"
  "float32 voltage_v\\t\\t\\t\\t# Battery voltage in volts, 0 if unknown\n"
  "float32 current_a\\t\\t\\t\\t# Battery current in amperes, -1 if unknown\n"
  "float32 current_average_a\\t\\t\\t# Battery current average in amperes (for FW average in level flight), -1 if unknown\n"
  "float32 discharged_mah\\t\\t\\t\\t# Discharged amount in mAh, -1 if unknown\n"
  "float32 remaining\\t\\t\\t\\t# From 1 to 0, -1 if unknown\n"
  "float32 scale\\t\\t\\t\\t\\t# Power scaling factor, >= 1, or -1 if unknown\n"
  "float32 time_remaining_s\\t\\t\\t# predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown\n"
  "float32 temperature\\t\\t\\t\\t# Temperature of the battery in degrees Celcius, NaN if unknown\n"
  "uint8 cell_count\\t\\t\\t\\t# Number of cells, 0 if unknown\n"
  "\n"
  "uint8 SOURCE_POWER_MODULE = 0\n"
  "uint8 SOURCE_EXTERNAL = 1\n"
  "uint8 SOURCE_ESCS = 2\n"
  "uint8 source\\t\\t\\t\\t\\t# Battery source\n"
  "uint8 priority\\t\\t\\t\\t\\t# Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1\n"
  "uint16 capacity\\t\\t\\t\\t\\t# actual capacity of the battery\n"
  "uint16 cycle_count\\t\\t\\t\\t# number of discharge cycles the battery has experienced\n"
  "uint16 average_time_to_empty\\t\\t\\t# predicted remaining battery capacity based on the average rate of discharge in min\n"
  "uint16 serial_number\\t\\t\\t\\t# serial number of the battery pack\n"
  "uint16 manufacture_date\\t\\t\\t\\t# manufacture date, part of serial number of the battery pack. Formatted as: Day + Month\\xc3\\x9732 + (Year\\xe2\\x80\\x931980)\\xc3\\x97512\n"
  "uint16 state_of_health\\t\\t\\t\\t# state of health. FullChargeCapacity/DesignCapacity, 0-100%.\n"
  "uint16 max_error\\t\\t\\t\\t# max error, expected margin of error in % in the state-of-charge calculation with a range of 1 to 100%\n"
  "uint8 id\\t\\t\\t\\t\\t# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.\n"
  "uint16 interface_error\\t\\t\\t\\t# interface error counter\n"
  "\n"
  "float32[14] voltage_cell_v\\t\\t\\t# Battery individual cell voltages, 0 if unknown\n"
  "float32 max_cell_voltage_delta\\t\\t\\t# Max difference between individual cell voltages\n"
  "\n"
  "bool is_powering_off\\t\\t\\t\\t# Power off event imminent indication, false if unknown\n"
  "bool is_required\\t\\t\\t\\t# Set if the battery is explicitly required before arming\n"
  "\n"
  "\n"
  "uint8 WARNING_NONE = 0\\t\\t\\t# no battery low voltage warning active\n"
  "uint8 WARNING_LOW = 1\\t\\t\\t# warning of low voltage\n"
  "uint8 WARNING_CRITICAL = 2\\t\\t# critical voltage, return / abort immediately\n"
  "uint8 WARNING_EMERGENCY = 3\\t\\t# immediate landing required\n"
  "uint8 WARNING_FAILED = 4\\t\\t# the battery has failed completely\n"
  "uint8 STATE_UNHEALTHY = 6 \\t\\t# Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field.\n"
  "uint8 STATE_CHARGING = 7 \\t\\t# Battery is charging\n"
  "\n"
  "uint8 FAULT_DEEP_DISCHARGE = 0 \\t\\t# Battery has deep discharged\n"
  "uint8 FAULT_SPIKES = 1 \\t\\t\\t# Voltage spikes\n"
  "uint8 FAULT_CELL_FAIL= 2 \\t\\t# One or more cells have failed\n"
  "uint8 FAULT_OVER_CURRENT = 3 \\t\\t# Over-current\n"
  "uint8 FAULT_OVER_TEMPERATURE = 4 \\t# Over-temperature\n"
  "uint8 FAULT_UNDER_TEMPERATURE = 5 \\t# Under-temperature fault\n"
  "uint8 FAULT_INCOMPATIBLE_VOLTAGE = 6 \\t# Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage).\n"
  "uint8 FAULT_INCOMPATIBLE_FIRMWARE = 7 \\t# Battery firmware is not compatible with current autopilot firmware\n"
  "uint8 FAULT_INCOMPATIBLE_MODEL = 8 \\t# Battery model is not supported by the system\n"
  "uint8 FAULT_HARDWARE_FAILURE = 9 \\t# hardware problem\n"
  "uint8 FAULT_FAILED_TO_ARM = 10 \\t\\t# Battery had a problem while arming\n"
  "uint8 FAULT_COUNT = 11 \\t\\t\\t# Counter - keep it as last element!\n"
  "\n"
  "uint16 faults\\t\\t\\t\\t\\t# Smart battery supply status/fault flags (bitmask) for health indication.\n"
  "uint8 warning\\t\\t\\t\\t\\t# Current battery warning\n"
  "\n"
  "uint8 MAX_INSTANCES = 4\n"
  "\n"
  "float32 full_charge_capacity_wh     \\t\\t# The compensated battery capacity\n"
  "float32 remaining_capacity_wh       \\t\\t# The compensated battery capacity remaining\n"
  "uint16 over_discharge_count         \\t\\t# Number of battery overdischarge\n"
  "float32 nominal_voltage             \\t\\t# Nominal voltage of the battery pack\n"
  "\n"
  "float32 internal_resistance_estimate            # [Ohm] Internal resistance per cell estimate\n"
  "float32 ocv_estimate                            # [V] Open circuit voltage estimate\n"
  "float32 ocv_estimate_filtered\\t\\t\\t# [V] Filtered open circuit voltage estimate\n"
  "float32 volt_based_soc_estimate\\t\\t\\t# [0, 1] Normalized volt based state of charge estimate\n"
  "float32 voltage_prediction\\t\\t\\t# [V] Predicted voltage\n"
  "float32 prediction_error                        # [V] Prediction error\n"
  "float32 estimation_covariance_norm\\t\\t# Norm of the covariance matrix";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__BatteryStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__BatteryStatus__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 4630, 4630},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__BatteryStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__BatteryStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
