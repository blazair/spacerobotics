// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice

#ifndef CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__TRAITS_HPP_
#define CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control/msg/detail/performance_metrics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace control
{

namespace msg
{

inline void to_flow_style_yaml(
  const PerformanceMetrics & msg,
  std::ostream & out)
{
  out << "{";
  // member: cross_track_error
  {
    out << "cross_track_error: ";
    rosidl_generator_traits::value_to_yaml(msg.cross_track_error, out);
    out << ", ";
  }

  // member: current_velocity
  {
    out << "current_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity, out);
    out << ", ";
  }

  // member: distance_to_next_waypoint
  {
    out << "distance_to_next_waypoint: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_next_waypoint, out);
    out << ", ";
  }

  // member: completion_percentage
  {
    out << "completion_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.completion_percentage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PerformanceMetrics & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cross_track_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cross_track_error: ";
    rosidl_generator_traits::value_to_yaml(msg.cross_track_error, out);
    out << "\n";
  }

  // member: current_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity, out);
    out << "\n";
  }

  // member: distance_to_next_waypoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_to_next_waypoint: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_next_waypoint, out);
    out << "\n";
  }

  // member: completion_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "completion_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.completion_percentage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PerformanceMetrics & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace control

namespace rosidl_generator_traits
{

[[deprecated("use control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control::msg::PerformanceMetrics & msg,
  std::ostream & out, size_t indentation = 0)
{
  control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control::msg::to_yaml() instead")]]
inline std::string to_yaml(const control::msg::PerformanceMetrics & msg)
{
  return control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<control::msg::PerformanceMetrics>()
{
  return "control::msg::PerformanceMetrics";
}

template<>
inline const char * name<control::msg::PerformanceMetrics>()
{
  return "control/msg/PerformanceMetrics";
}

template<>
struct has_fixed_size<control::msg::PerformanceMetrics>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<control::msg::PerformanceMetrics>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<control::msg::PerformanceMetrics>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__TRAITS_HPP_
