// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice

#ifndef CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__STRUCT_HPP_
#define CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__control__msg__PerformanceMetrics __attribute__((deprecated))
#else
# define DEPRECATED__control__msg__PerformanceMetrics __declspec(deprecated)
#endif

namespace control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PerformanceMetrics_
{
  using Type = PerformanceMetrics_<ContainerAllocator>;

  explicit PerformanceMetrics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cross_track_error = 0.0;
      this->current_velocity = 0.0;
      this->distance_to_next_waypoint = 0.0;
      this->completion_percentage = 0.0;
    }
  }

  explicit PerformanceMetrics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cross_track_error = 0.0;
      this->current_velocity = 0.0;
      this->distance_to_next_waypoint = 0.0;
      this->completion_percentage = 0.0;
    }
  }

  // field types and members
  using _cross_track_error_type =
    double;
  _cross_track_error_type cross_track_error;
  using _current_velocity_type =
    double;
  _current_velocity_type current_velocity;
  using _distance_to_next_waypoint_type =
    double;
  _distance_to_next_waypoint_type distance_to_next_waypoint;
  using _completion_percentage_type =
    double;
  _completion_percentage_type completion_percentage;

  // setters for named parameter idiom
  Type & set__cross_track_error(
    const double & _arg)
  {
    this->cross_track_error = _arg;
    return *this;
  }
  Type & set__current_velocity(
    const double & _arg)
  {
    this->current_velocity = _arg;
    return *this;
  }
  Type & set__distance_to_next_waypoint(
    const double & _arg)
  {
    this->distance_to_next_waypoint = _arg;
    return *this;
  }
  Type & set__completion_percentage(
    const double & _arg)
  {
    this->completion_percentage = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control::msg::PerformanceMetrics_<ContainerAllocator> *;
  using ConstRawPtr =
    const control::msg::PerformanceMetrics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control::msg::PerformanceMetrics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control::msg::PerformanceMetrics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control::msg::PerformanceMetrics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control::msg::PerformanceMetrics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control::msg::PerformanceMetrics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control::msg::PerformanceMetrics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control::msg::PerformanceMetrics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control::msg::PerformanceMetrics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control__msg__PerformanceMetrics
    std::shared_ptr<control::msg::PerformanceMetrics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control__msg__PerformanceMetrics
    std::shared_ptr<control::msg::PerformanceMetrics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerformanceMetrics_ & other) const
  {
    if (this->cross_track_error != other.cross_track_error) {
      return false;
    }
    if (this->current_velocity != other.current_velocity) {
      return false;
    }
    if (this->distance_to_next_waypoint != other.distance_to_next_waypoint) {
      return false;
    }
    if (this->completion_percentage != other.completion_percentage) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerformanceMetrics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerformanceMetrics_

// alias to use template instance with default allocator
using PerformanceMetrics =
  control::msg::PerformanceMetrics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control

#endif  // CONTROL__MSG__DETAIL__PERFORMANCE_METRICS__STRUCT_HPP_
