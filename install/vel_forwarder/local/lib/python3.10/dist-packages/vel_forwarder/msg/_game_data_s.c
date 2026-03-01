// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vel_forwarder:msg/GameData.idl
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
#include "vel_forwarder/msg/detail/game_data__struct.h"
#include "vel_forwarder/msg/detail/game_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vel_forwarder__msg__game_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("vel_forwarder.msg._game_data.GameData", full_classname_dest, 37) == 0);
  }
  vel_forwarder__msg__GameData * ros_message = _ros_message;
  {  // gamecode
    PyObject * field = PyObject_GetAttrString(_pymsg, "gamecode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gamecode = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ammoleft
    PyObject * field = PyObject_GetAttrString(_pymsg, "ammoleft");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ammoleft = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timeleft
    PyObject * field = PyObject_GetAttrString(_pymsg, "timeleft");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timeleft = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // selfhealth
    PyObject * field = PyObject_GetAttrString(_pymsg, "selfhealth");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->selfhealth = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // exteventdata
    PyObject * field = PyObject_GetAttrString(_pymsg, "exteventdata");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->exteventdata = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vel_forwarder__msg__game_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GameData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vel_forwarder.msg._game_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GameData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vel_forwarder__msg__GameData * ros_message = (vel_forwarder__msg__GameData *)raw_ros_message;
  {  // gamecode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->gamecode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gamecode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ammoleft
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ammoleft);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ammoleft", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timeleft
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timeleft);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timeleft", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selfhealth
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->selfhealth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selfhealth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // exteventdata
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->exteventdata);
    {
      int rc = PyObject_SetAttrString(_pymessage, "exteventdata", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
