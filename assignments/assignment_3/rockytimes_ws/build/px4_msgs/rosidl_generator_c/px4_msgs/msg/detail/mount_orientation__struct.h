// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/MountOrientation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/msg/mount_orientation.h"


#ifndef PX4_MSGS__MSG__DETAIL__MOUNT_ORIENTATION__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__MOUNT_ORIENTATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MountOrientation in the package px4_msgs.
typedef struct px4_msgs__msg__MountOrientation
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Attitude/direction of the mount as euler angles in rad
  float attitude_euler_angle[3];
} px4_msgs__msg__MountOrientation;

// Struct for a sequence of px4_msgs__msg__MountOrientation.
typedef struct px4_msgs__msg__MountOrientation__Sequence
{
  px4_msgs__msg__MountOrientation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__MountOrientation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__MOUNT_ORIENTATION__STRUCT_H_
