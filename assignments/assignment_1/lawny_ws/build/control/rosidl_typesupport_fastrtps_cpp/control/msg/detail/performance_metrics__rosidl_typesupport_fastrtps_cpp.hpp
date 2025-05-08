// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice

#ifndef CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "control/msg/detail/performance_metrics__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace control
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_control
cdr_serialize(
  const control::msg::PerformanceMetrics & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  control::msg::PerformanceMetrics & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_control
get_serialized_size(
  const control::msg::PerformanceMetrics & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_control
max_serialized_size_PerformanceMetrics(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace control

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control, msg, PerformanceMetrics)();

#ifdef __cplusplus
}
#endif

#endif  // CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
