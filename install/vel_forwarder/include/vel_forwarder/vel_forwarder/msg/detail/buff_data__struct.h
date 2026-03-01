// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vel_forwarder:msg/BuffData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__STRUCT_H_
#define VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BuffData in the package vel_forwarder.
/**
  * 我队增益类型
 */
typedef struct vel_forwarder__msg__BuffData
{
  uint8_t recoverybuff;
  uint8_t coolingbuff;
  uint8_t defencebuff;
  uint8_t vulnerabilitybuff;
  uint16_t attackbuff;
  uint8_t remainingenergy;
} vel_forwarder__msg__BuffData;

// Struct for a sequence of vel_forwarder__msg__BuffData.
typedef struct vel_forwarder__msg__BuffData__Sequence
{
  vel_forwarder__msg__BuffData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vel_forwarder__msg__BuffData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__STRUCT_H_
