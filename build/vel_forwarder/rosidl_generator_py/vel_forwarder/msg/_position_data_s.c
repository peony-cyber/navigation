// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vel_forwarder:msg/PositionData.idl
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
#include "vel_forwarder/msg/detail/position_data__struct.h"
#include "vel_forwarder/msg/detail/position_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vel_forwarder__msg__position_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("vel_forwarder.msg._position_data.PositionData", full_classname_dest, 45) == 0);
  }
  vel_forwarder__msg__PositionData * ros_message = _ros_message;
  {  // friendcarid
    PyObject * field = PyObject_GetAttrString(_pymsg, "friendcarid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->friendcarid = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // friendx
    PyObject * field = PyObject_GetAttrString(_pymsg, "friendx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->friendx = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // friendy
    PyObject * field = PyObject_GetAttrString(_pymsg, "friendy");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->friendy = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemycarid
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemycarid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemycarid = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // enemyx
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemyx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemyx = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemyy
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemyy");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemyy = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vel_forwarder__msg__position_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PositionData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vel_forwarder.msg._position_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PositionData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vel_forwarder__msg__PositionData * ros_message = (vel_forwarder__msg__PositionData *)raw_ros_message;
  {  // friendcarid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->friendcarid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "friendcarid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // friendx
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->friendx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "friendx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // friendy
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->friendy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "friendy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemycarid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->enemycarid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemycarid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemyx
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemyx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemyx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemyy
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemyy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemyy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
