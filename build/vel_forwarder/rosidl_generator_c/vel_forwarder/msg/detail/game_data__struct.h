// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__GAME_DATA__STRUCT_H_
#define VEL_FORWARDER__MSG__DETAIL__GAME_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GameData in the package vel_forwarder.
typedef struct vel_forwarder__msg__GameData
{
  uint16_t gamecode;
  uint16_t ammoleft;
  uint16_t timeleft;
  uint16_t selfhealth;
  uint32_t exteventdata;
} vel_forwarder__msg__GameData;

// Struct for a sequence of vel_forwarder__msg__GameData.
typedef struct vel_forwarder__msg__GameData__Sequence
{
  vel_forwarder__msg__GameData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vel_forwarder__msg__GameData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__GAME_DATA__STRUCT_H_
