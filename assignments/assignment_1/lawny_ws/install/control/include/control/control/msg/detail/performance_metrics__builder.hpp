// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice

#ifndef CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__BUILDER_HPP_
#define CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control/msg/detail/performance_metrics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control
{

namespace msg
{

namespace builder
{

class Init_PerformanceMetrics_completion_percentage
{
public:
  explicit Init_PerformanceMetrics_completion_percentage(::control::msg::PerformanceMetrics & msg)
  : msg_(msg)
  {}
  ::control::msg::PerformanceMetrics completion_percentage(::control::msg::PerformanceMetrics::_completion_percentage_type arg)
  {
    msg_.completion_percentage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control::msg::PerformanceMetrics msg_;
};

class Init_PerformanceMetrics_distance_to_next_waypoint
{
public:
  explicit Init_PerformanceMetrics_distance_to_next_waypoint(::control::msg::PerformanceMetrics & msg)
  : msg_(msg)
  {}
  Init_PerformanceMetrics_completion_percentage distance_to_next_waypoint(::control::msg::PerformanceMetrics::_distance_to_next_waypoint_type arg)
  {
    msg_.distance_to_next_waypoint = std::move(arg);
    return Init_PerformanceMetrics_completion_percentage(msg_);
  }

private:
  ::control::msg::PerformanceMetrics msg_;
};

class Init_PerformanceMetrics_current_velocity
{
public:
  explicit Init_PerformanceMetrics_current_velocity(::control::msg::PerformanceMetrics & msg)
  : msg_(msg)
  {}
  Init_PerformanceMetrics_distance_to_next_waypoint current_velocity(::control::msg::PerformanceMetrics::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_PerformanceMetrics_distance_to_next_waypoint(msg_);
  }

private:
  ::control::msg::PerformanceMetrics msg_;
};

class Init_PerformanceMetrics_cross_track_error
{
public:
  Init_PerformanceMetrics_cross_track_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformanceMetrics_current_velocity cross_track_error(::control::msg::PerformanceMetrics::_cross_track_error_type arg)
  {
    msg_.cross_track_error = std::move(arg);
    return Init_PerformanceMetrics_current_velocity(msg_);
  }

private:
  ::control::msg::PerformanceMetrics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control::msg::PerformanceMetrics>()
{
  return control::msg::builder::Init_PerformanceMetrics_cross_track_error();
}

}  // namespace control

#endif  // CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__BUILDER_HPP_
