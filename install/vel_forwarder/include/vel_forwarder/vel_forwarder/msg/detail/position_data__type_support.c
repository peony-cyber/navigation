// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vel_forwarder/msg/detail/position_data__rosidl_typesupport_introspection_c.h"
#include "vel_forwarder/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vel_forwarder/msg/detail/position_data__functions.h"
#include "vel_forwarder/msg/detail/position_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vel_forwarder__msg__PositionData__init(message_memory);
}

void vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_fini_function(void * message_memory)
{
  vel_forwarder__msg__PositionData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_member_array[6] = {
  {
    "friendcarid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__PositionData, friendcarid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "friendx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__PositionData, friendx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "friendy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__PositionData, friendy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemycarid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__PositionData, enemycarid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemyx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__PositionData, enemyx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemyy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__PositionData, enemyy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_members = {
  "vel_forwarder__msg",  // message namespace
  "PositionData",  // message name
  6,  // number of fields
  sizeof(vel_forwarder__msg__PositionData),
  vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_member_array,  // message members
  vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_init_function,  // function to initialize message memory (memory has to be allocated)
  vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_type_support_handle = {
  0,
  &vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vel_forwarder
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vel_forwarder, msg, PositionData)() {
  if (!vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_type_support_handle.typesupport_identifier) {
    vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vel_forwarder__msg__PositionData__rosidl_typesupport_introspection_c__PositionData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
