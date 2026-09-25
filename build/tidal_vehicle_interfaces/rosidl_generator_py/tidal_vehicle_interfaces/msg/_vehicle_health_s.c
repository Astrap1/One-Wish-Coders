// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
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
#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__struct.h"
#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool tidal_vehicle_interfaces__msg__vehicle_health__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[59];
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
    assert(strncmp("tidal_vehicle_interfaces.msg._vehicle_health.VehicleHealth", full_classname_dest, 58) == 0);
  }
  tidal_vehicle_interfaces__msg__VehicleHealth * ros_message = _ros_message;
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
  {  // battery_percent
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_percent");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_percent = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // return_reserve_percent
    PyObject * field = PyObject_GetAttrString(_pymsg, "return_reserve_percent");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->return_reserve_percent = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mobility_health_percent
    PyObject * field = PyObject_GetAttrString(_pymsg, "mobility_health_percent");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mobility_health_percent = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // link_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "link_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->link_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // payload_secured
    PyObject * field = PyObject_GetAttrString(_pymsg, "payload_secured");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->payload_secured = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->fault, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * tidal_vehicle_interfaces__msg__vehicle_health__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VehicleHealth */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("tidal_vehicle_interfaces.msg._vehicle_health");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VehicleHealth");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  tidal_vehicle_interfaces__msg__VehicleHealth * ros_message = (tidal_vehicle_interfaces__msg__VehicleHealth *)raw_ros_message;
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
  {  // battery_percent
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_percent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_percent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // return_reserve_percent
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->return_reserve_percent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "return_reserve_percent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mobility_health_percent
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mobility_health_percent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mobility_health_percent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // link_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->link_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "link_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // payload_secured
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->payload_secured ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "payload_secured", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->fault.data,
      strlen(ros_message->fault.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
