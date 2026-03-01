// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__STRUCT_H_
#define VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PositionData in the package vel_forwarder.
/**
  * 位置消息
 */
typedef struct vel_forwarder__msg__PositionData
{
  uint8_t friendcarid;
  int16_t friendx;
  int16_t friendy;
  uint8_t enemycarid;
  int16_t enemyx;
  int16_t enemyy;
} vel_forwarder__msg__PositionData;

// Struct for a sequence of vel_forwarder__msg__PositionData.
typedef struct vel_forwarder__msg__PositionData__Sequence
{
  vel_forwarder__msg__PositionData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vel_forwarder__msg__PositionData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__STRUCT_H_
