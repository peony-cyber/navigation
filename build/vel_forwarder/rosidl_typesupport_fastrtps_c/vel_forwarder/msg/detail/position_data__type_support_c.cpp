// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/position_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vel_forwarder/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vel_forwarder/msg/detail/position_data__struct.h"
#include "vel_forwarder/msg/detail/position_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _PositionData__ros_msg_type = vel_forwarder__msg__PositionData;

static bool _PositionData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PositionData__ros_msg_type * ros_message = static_cast<const _PositionData__ros_msg_type *>(untyped_ros_message);
  // Field name: friendcarid
  {
    cdr << ros_message->friendcarid;
  }

  // Field name: friendx
  {
    cdr << ros_message->friendx;
  }

  // Field name: friendy
  {
    cdr << ros_message->friendy;
  }

  // Field name: enemycarid
  {
    cdr << ros_message->enemycarid;
  }

  // Field name: enemyx
  {
    cdr << ros_message->enemyx;
  }

  // Field name: enemyy
  {
    cdr << ros_message->enemyy;
  }

  return true;
}

static bool _PositionData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PositionData__ros_msg_type * ros_message = static_cast<_PositionData__ros_msg_type *>(untyped_ros_message);
  // Field name: friendcarid
  {
    cdr >> ros_message->friendcarid;
  }

  // Field name: friendx
  {
    cdr >> ros_message->friendx;
  }

  // Field name: friendy
  {
    cdr >> ros_message->friendy;
  }

  // Field name: enemycarid
  {
    cdr >> ros_message->enemycarid;
  }

  // Field name: enemyx
  {
    cdr >> ros_message->enemyx;
  }

  // Field name: enemyy
  {
    cdr >> ros_message->enemyy;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vel_forwarder
size_t get_serialized_size_vel_forwarder__msg__PositionData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PositionData__ros_msg_type * ros_message = static_cast<const _PositionData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name friendcarid
  {
    size_t item_size = sizeof(ros_message->friendcarid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name friendx
  {
    size_t item_size = sizeof(ros_message->friendx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name friendy
  {
    size_t item_size = sizeof(ros_message->friendy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enemycarid
  {
    size_t item_size = sizeof(ros_message->enemycarid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enemyx
  {
    size_t item_size = sizeof(ros_message->enemyx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enemyy
  {
    size_t item_size = sizeof(ros_message->enemyy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PositionData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vel_forwarder__msg__PositionData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vel_forwarder
size_t max_serialized_size_vel_forwarder__msg__PositionData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: friendcarid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: friendx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: friendy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: enemycarid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: enemyx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: enemyy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vel_forwarder__msg__PositionData;
    is_plain =
      (
      offsetof(DataType, enemyy) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PositionData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vel_forwarder__msg__PositionData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PositionData = {
  "vel_forwarder::msg",
  "PositionData",
  _PositionData__cdr_serialize,
  _PositionData__cdr_deserialize,
  _PositionData__get_serialized_size,
  _PositionData__max_serialized_size
};

static rosidl_message_type_support_t _PositionData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PositionData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vel_forwarder, msg, PositionData)() {
  return &_PositionData__type_support;
}

#if defined(__cplusplus)
}
#endif
