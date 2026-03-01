// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vel_forwarder:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__HEALTH__STRUCT_H_
#define VEL_FORWARDER__MSG__DETAIL__HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Health in the package vel_forwarder.
typedef struct vel_forwarder__msg__Health
{
  uint16_t hero;
  uint16_t engineer;
  uint16_t infantry1;
  uint16_t infantry2;
  uint16_t reserve;
  uint16_t sentry;
} vel_forwarder__msg__Health;

// Struct for a sequence of vel_forwarder__msg__Health.
typedef struct vel_forwarder__msg__Health__Sequence
{
  vel_forwarder__msg__Health * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vel_forwarder__msg__Health__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__HEALTH__STRUCT_H_
