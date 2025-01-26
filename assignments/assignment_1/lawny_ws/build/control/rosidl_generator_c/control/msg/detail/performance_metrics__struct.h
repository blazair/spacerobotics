// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice

#ifndef CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__STRUCT_H_
#define CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PerformanceMetrics in the package control.
typedef struct control__msg__PerformanceMetrics
{
  double cross_track_error;
  double current_velocity;
  double distance_to_next_waypoint;
  double completion_percentage;
} control__msg__PerformanceMetrics;

// Struct for a sequence of control__msg__PerformanceMetrics.
typedef struct control__msg__PerformanceMetrics__Sequence
{
  control__msg__PerformanceMetrics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control__msg__PerformanceMetrics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__STRUCT_H_
