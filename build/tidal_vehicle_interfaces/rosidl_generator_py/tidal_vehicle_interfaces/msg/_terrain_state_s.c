// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
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
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__struct.h"
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool tidal_vehicle_interfaces__msg__terrain_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
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
    assert(strncmp("tidal_vehicle_interfaces.msg._terrain_state.TerrainState", full_classname_dest, 56) == 0);
  }
  tidal_vehicle_interfaces__msg__TerrainState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // tide_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "tide_state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->tide_state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // tide_risk
    PyObject * field = PyObject_GetAttrString(_pymsg, "tide_risk");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tide_risk = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // water_level_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "water_level_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->water_level_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tide_rate_m_per_minute
    PyObject * field = PyObject_GetAttrString(_pymsg, "tide_rate_m_per_minute");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tide_rate_m_per_minute = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // seconds_until_corridor_unsafe
    PyObject * field = PyObject_GetAttrString(_pymsg, "seconds_until_corridor_unsafe");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->seconds_until_corridor_unsafe = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // corridor_traversable
    PyObject * field = PyObject_GetAttrString(_pymsg, "corridor_traversable");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->corridor_traversable = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * tidal_vehicle_interfaces__msg__terrain_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TerrainState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("tidal_vehicle_interfaces.msg._terrain_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TerrainState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  tidal_vehicle_interfaces__msg__TerrainState * ros_message = (tidal_vehicle_interfaces__msg__TerrainState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tide_state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->tide_state.data,
      strlen(ros_message->tide_state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "tide_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tide_risk
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tide_risk);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tide_risk", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // water_level_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->water_level_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "water_level_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tide_rate_m_per_minute
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tide_rate_m_per_minute);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tide_rate_m_per_minute", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // seconds_until_corridor_unsafe
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->seconds_until_corridor_unsafe);
    {
      int rc = PyObject_SetAttrString(_pymessage, "seconds_until_corridor_unsafe", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // corridor_traversable
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->corridor_traversable ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "corridor_traversable", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
