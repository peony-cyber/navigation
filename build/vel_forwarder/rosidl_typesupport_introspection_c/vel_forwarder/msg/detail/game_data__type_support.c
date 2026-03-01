// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vel_forwarder/msg/detail/game_data__rosidl_typesupport_introspection_c.h"
#include "vel_forwarder/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vel_forwarder/msg/detail/game_data__functions.h"
#include "vel_forwarder/msg/detail/game_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vel_forwarder__msg__GameData__init(message_memory);
}

void vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_fini_function(void * message_memory)
{
  vel_forwarder__msg__GameData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_member_array[5] = {
  {
    "gamecode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__GameData, gamecode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ammoleft",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__GameData, ammoleft),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timeleft",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__GameData, timeleft),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selfhealth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__GameData, selfhealth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "exteventdata",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vel_forwarder__msg__GameData, exteventdata),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_members = {
  "vel_forwarder__msg",  // message namespace
  "GameData",  // message name
  5,  // number of fields
  sizeof(vel_forwarder__msg__GameData),
  vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_member_array,  // message members
  vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_init_function,  // function to initialize message memory (memory has to be allocated)
  vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle = {
  0,
  &vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vel_forwarder
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vel_forwarder, msg, GameData)() {
  if (!vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle.typesupport_identifier) {
    vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vel_forwarder__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
