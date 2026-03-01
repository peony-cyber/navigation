// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vel_forwarder:msg/Vel.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__VEL__STRUCT_H_
#define VEL_FORWARDER__MSG__DETAIL__VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Vel in the package vel_forwarder.
typedef struct vel_forwarder__msg__Vel
{
  int8_t x;
  int8_t y;
} vel_forwarder__msg__Vel;

// Struct for a sequence of vel_forwarder__msg__Vel.
typedef struct vel_forwarder__msg__Vel__Sequence
{
  vel_forwarder__msg__Vel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vel_forwarder__msg__Vel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__VEL__STRUCT_H_
