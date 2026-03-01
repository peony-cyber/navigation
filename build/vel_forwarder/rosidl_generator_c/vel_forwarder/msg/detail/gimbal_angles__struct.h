// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vel_forwarder:msg/GimbalAngles.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__GIMBAL_ANGLES__STRUCT_H_
#define VEL_FORWARDER__MSG__DETAIL__GIMBAL_ANGLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GimbalAngles in the package vel_forwarder.
typedef struct vel_forwarder__msg__GimbalAngles
{
  float yaw;
  float pitch;
  std_msgs__msg__Header header;
} vel_forwarder__msg__GimbalAngles;

// Struct for a sequence of vel_forwarder__msg__GimbalAngles.
typedef struct vel_forwarder__msg__GimbalAngles__Sequence
{
  vel_forwarder__msg__GimbalAngles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vel_forwarder__msg__GimbalAngles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__GIMBAL_ANGLES__STRUCT_H_
