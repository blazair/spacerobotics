// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from control:msg/PerformanceMetrics.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "control/msg/detail/performance_metrics__struct.h"
#include "control/msg/detail/performance_metrics__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool control__msg__performance_metrics__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("control.msg._performance_metrics.PerformanceMetrics", full_classname_dest, 51) == 0);
  }
  control__msg__PerformanceMetrics * ros_message = _ros_message;
  {  // cross_track_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "cross_track_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cross_track_error = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance_to_next_waypoint
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_to_next_waypoint");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_to_next_waypoint = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // completion_percentage
    PyObject * field = PyObject_GetAttrString(_pymsg, "completion_percentage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->completion_percentage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * control__msg__performance_metrics__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PerformanceMetrics */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("control.msg._performance_metrics");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PerformanceMetrics");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  control__msg__PerformanceMetrics * ros_message = (control__msg__PerformanceMetrics *)raw_ros_message;
  {  // cross_track_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cross_track_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cross_track_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_to_next_waypoint
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_to_next_waypoint);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_to_next_waypoint", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // completion_percentage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->completion_percentage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "completion_percentage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
