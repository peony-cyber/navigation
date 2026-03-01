// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vel_forwarder:msg/BuffData.idl
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
#include "vel_forwarder/msg/detail/buff_data__struct.h"
#include "vel_forwarder/msg/detail/buff_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vel_forwarder__msg__buff_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("vel_forwarder.msg._buff_data.BuffData", full_classname_dest, 37) == 0);
  }
  vel_forwarder__msg__BuffData * ros_message = _ros_message;
  {  // recoverybuff
    PyObject * field = PyObject_GetAttrString(_pymsg, "recoverybuff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->recoverybuff = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // coolingbuff
    PyObject * field = PyObject_GetAttrString(_pymsg, "coolingbuff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->coolingbuff = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // defencebuff
    PyObject * field = PyObject_GetAttrString(_pymsg, "defencebuff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->defencebuff = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // vulnerabilitybuff
    PyObject * field = PyObject_GetAttrString(_pymsg, "vulnerabilitybuff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vulnerabilitybuff = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // attackbuff
    PyObject * field = PyObject_GetAttrString(_pymsg, "attackbuff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->attackbuff = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // remainingenergy
    PyObject * field = PyObject_GetAttrString(_pymsg, "remainingenergy");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->remainingenergy = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vel_forwarder__msg__buff_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BuffData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vel_forwarder.msg._buff_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BuffData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vel_forwarder__msg__BuffData * ros_message = (vel_forwarder__msg__BuffData *)raw_ros_message;
  {  // recoverybuff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->recoverybuff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "recoverybuff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // coolingbuff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->coolingbuff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "coolingbuff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // defencebuff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->defencebuff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "defencebuff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vulnerabilitybuff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->vulnerabilitybuff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vulnerabilitybuff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // attackbuff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->attackbuff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attackbuff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // remainingenergy
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->remainingenergy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "remainingenergy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
