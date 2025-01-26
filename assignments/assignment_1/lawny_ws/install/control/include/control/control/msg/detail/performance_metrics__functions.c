// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice
#include "control/msg/detail/performance_metrics__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
control__msg__PerformanceMetrics__init(control__msg__PerformanceMetrics * msg)
{
  if (!msg) {
    return false;
  }
  // cross_track_error
  // current_velocity
  // distance_to_next_waypoint
  // completion_percentage
  return true;
}

void
control__msg__PerformanceMetrics__fini(control__msg__PerformanceMetrics * msg)
{
  if (!msg) {
    return;
  }
  // cross_track_error
  // current_velocity
  // distance_to_next_waypoint
  // completion_percentage
}

bool
control__msg__PerformanceMetrics__are_equal(const control__msg__PerformanceMetrics * lhs, const control__msg__PerformanceMetrics * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cross_track_error
  if (lhs->cross_track_error != rhs->cross_track_error) {
    return false;
  }
  // current_velocity
  if (lhs->current_velocity != rhs->current_velocity) {
    return false;
  }
  // distance_to_next_waypoint
  if (lhs->distance_to_next_waypoint != rhs->distance_to_next_waypoint) {
    return false;
  }
  // completion_percentage
  if (lhs->completion_percentage != rhs->completion_percentage) {
    return false;
  }
  return true;
}

bool
control__msg__PerformanceMetrics__copy(
  const control__msg__PerformanceMetrics * input,
  control__msg__PerformanceMetrics * output)
{
  if (!input || !output) {
    return false;
  }
  // cross_track_error
  output->cross_track_error = input->cross_track_error;
  // current_velocity
  output->current_velocity = input->current_velocity;
  // distance_to_next_waypoint
  output->distance_to_next_waypoint = input->distance_to_next_waypoint;
  // completion_percentage
  output->completion_percentage = input->completion_percentage;
  return true;
}

control__msg__PerformanceMetrics *
control__msg__PerformanceMetrics__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control__msg__PerformanceMetrics * msg = (control__msg__PerformanceMetrics *)allocator.allocate(sizeof(control__msg__PerformanceMetrics), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(control__msg__PerformanceMetrics));
  bool success = control__msg__PerformanceMetrics__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
control__msg__PerformanceMetrics__destroy(control__msg__PerformanceMetrics * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    control__msg__PerformanceMetrics__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
control__msg__PerformanceMetrics__Sequence__init(control__msg__PerformanceMetrics__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control__msg__PerformanceMetrics * data = NULL;

  if (size) {
    data = (control__msg__PerformanceMetrics *)allocator.zero_allocate(size, sizeof(control__msg__PerformanceMetrics), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = control__msg__PerformanceMetrics__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        control__msg__PerformanceMetrics__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
control__msg__PerformanceMetrics__Sequence__fini(control__msg__PerformanceMetrics__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      control__msg__PerformanceMetrics__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

control__msg__PerformanceMetrics__Sequence *
control__msg__PerformanceMetrics__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control__msg__PerformanceMetrics__Sequence * array = (control__msg__PerformanceMetrics__Sequence *)allocator.allocate(sizeof(control__msg__PerformanceMetrics__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = control__msg__PerformanceMetrics__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
control__msg__PerformanceMetrics__Sequence__destroy(control__msg__PerformanceMetrics__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    control__msg__PerformanceMetrics__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
control__msg__PerformanceMetrics__Sequence__are_equal(const control__msg__PerformanceMetrics__Sequence * lhs, const control__msg__PerformanceMetrics__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!control__msg__PerformanceMetrics__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
control__msg__PerformanceMetrics__Sequence__copy(
  const control__msg__PerformanceMetrics__Sequence * input,
  control__msg__PerformanceMetrics__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(control__msg__PerformanceMetrics);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    control__msg__PerformanceMetrics * data =
      (control__msg__PerformanceMetrics *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!control__msg__PerformanceMetrics__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          control__msg__PerformanceMetrics__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!control__msg__PerformanceMetrics__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
