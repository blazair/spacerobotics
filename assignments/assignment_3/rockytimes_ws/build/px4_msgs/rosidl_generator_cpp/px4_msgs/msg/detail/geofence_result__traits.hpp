// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/GeofenceResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/msg/geofence_result.hpp"


#ifndef PX4_MSGS__MSG__DETAIL__GEOFENCE_RESULT__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__GEOFENCE_RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/geofence_result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GeofenceResult & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: geofence_violation_reason
  {
    out << "geofence_violation_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.geofence_violation_reason, out);
    out << ", ";
  }

  // member: primary_geofence_breached
  {
    out << "primary_geofence_breached: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_geofence_breached, out);
    out << ", ";
  }

  // member: primary_geofence_action
  {
    out << "primary_geofence_action: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_geofence_action, out);
    out << ", ";
  }

  // member: home_required
  {
    out << "home_required: ";
    rosidl_generator_traits::value_to_yaml(msg.home_required, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GeofenceResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: geofence_violation_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geofence_violation_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.geofence_violation_reason, out);
    out << "\n";
  }

  // member: primary_geofence_breached
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "primary_geofence_breached: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_geofence_breached, out);
    out << "\n";
  }

  // member: primary_geofence_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "primary_geofence_action: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_geofence_action, out);
    out << "\n";
  }

  // member: home_required
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "home_required: ";
    rosidl_generator_traits::value_to_yaml(msg.home_required, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GeofenceResult & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::msg::GeofenceResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::GeofenceResult & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::GeofenceResult>()
{
  return "px4_msgs::msg::GeofenceResult";
}

template<>
inline const char * name<px4_msgs::msg::GeofenceResult>()
{
  return "px4_msgs/msg/GeofenceResult";
}

template<>
struct has_fixed_size<px4_msgs::msg::GeofenceResult>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::GeofenceResult>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::GeofenceResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__GEOFENCE_RESULT__TRAITS_HPP_
