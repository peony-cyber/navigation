// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vel_forwarder:msg/Health.idl
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
#include "vel_forwarder/msg/detail/health__struct.h"
#include "vel_forwarder/msg/detail/health__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vel_forwarder__msg__health__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("vel_forwarder.msg._health.Health", full_classname_dest, 32) == 0);
  }
  vel_forwarder__msg__Health * ros_message = _ros_message;
  {  // hero
    PyObject * field = PyObject_GetAttrString(_pymsg, "hero");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hero = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // engineer
    PyObject * field = PyObject_GetAttrString(_pymsg, "engineer");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->engineer = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // infantry1
    PyObject * field = PyObject_GetAttrString(_pymsg, "infantry1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->infantry1 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // infantry2
    PyObject * field = PyObject_GetAttrString(_pymsg, "infantry2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->infantry2 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // reserve
    PyObject * field = PyObject_GetAttrString(_pymsg, "reserve");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->reserve = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sentry
    PyObject * field = PyObject_GetAttrString(_pymsg, "sentry");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sentry = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vel_forwarder__msg__health__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Health */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vel_forwarder.msg._health");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Health");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vel_forwarder__msg__Health * ros_message = (vel_forwarder__msg__Health *)raw_ros_message;
  {  // hero
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->hero);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hero", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // engineer
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->engineer);
    {
      int rc = PyObject_SetAttrString(_pymessage, "engineer", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // infantry1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->infantry1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "infantry1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // infantry2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->infantry2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "infantry2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reserve
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->reserve);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reserve", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sentry
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sentry);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sentry", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
