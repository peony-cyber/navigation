// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vel_forwarder:msg/BuffData.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/buff_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vel_forwarder/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vel_forwarder/msg/detail/buff_data__struct.h"
#include "vel_forwarder/msg/detail/buff_data__functions.h"
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


using _BuffData__ros_msg_type = vel_forwarder__msg__BuffData;

static bool _BuffData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BuffData__ros_msg_type * ros_message = static_cast<const _BuffData__ros_msg_type *>(untyped_ros_message);
  // Field name: recoverybuff
  {
    cdr << ros_message->recoverybuff;
  }

  // Field name: coolingbuff
  {
    cdr << ros_message->coolingbuff;
  }

  // Field name: defencebuff
  {
    cdr << ros_message->defencebuff;
  }

  // Field name: vulnerabilitybuff
  {
    cdr << ros_message->vulnerabilitybuff;
  }

  // Field name: attackbuff
  {
    cdr << ros_message->attackbuff;
  }

  // Field name: remainingenergy
  {
    cdr << ros_message->remainingenergy;
  }

  return true;
}

static bool _BuffData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BuffData__ros_msg_type * ros_message = static_cast<_BuffData__ros_msg_type *>(untyped_ros_message);
  // Field name: recoverybuff
  {
    cdr >> ros_message->recoverybuff;
  }

  // Field name: coolingbuff
  {
    cdr >> ros_message->coolingbuff;
  }

  // Field name: defencebuff
  {
    cdr >> ros_message->defencebuff;
  }

  // Field name: vulnerabilitybuff
  {
    cdr >> ros_message->vulnerabilitybuff;
  }

  // Field name: attackbuff
  {
    cdr >> ros_message->attackbuff;
  }

  // Field name: remainingenergy
  {
    cdr >> ros_message->remainingenergy;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vel_forwarder
size_t get_serialized_size_vel_forwarder__msg__BuffData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BuffData__ros_msg_type * ros_message = static_cast<const _BuffData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name recoverybuff
  {
    size_t item_size = sizeof(ros_message->recoverybuff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name coolingbuff
  {
    size_t item_size = sizeof(ros_message->coolingbuff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name defencebuff
  {
    size_t item_size = sizeof(ros_message->defencebuff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vulnerabilitybuff
  {
    size_t item_size = sizeof(ros_message->vulnerabilitybuff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name attackbuff
  {
    size_t item_size = sizeof(ros_message->attackbuff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name remainingenergy
  {
    size_t item_size = sizeof(ros_message->remainingenergy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BuffData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vel_forwarder__msg__BuffData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vel_forwarder
size_t max_serialized_size_vel_forwarder__msg__BuffData(
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

  // member: recoverybuff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: coolingbuff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: defencebuff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: vulnerabilitybuff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: attackbuff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: remainingenergy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vel_forwarder__msg__BuffData;
    is_plain =
      (
      offsetof(DataType, remainingenergy) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _BuffData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vel_forwarder__msg__BuffData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BuffData = {
  "vel_forwarder::msg",
  "BuffData",
  _BuffData__cdr_serialize,
  _BuffData__cdr_deserialize,
  _BuffData__get_serialized_size,
  _BuffData__max_serialized_size
};

static rosidl_message_type_support_t _BuffData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BuffData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vel_forwarder, msg, BuffData)() {
  return &_BuffData__type_support;
}

#if defined(__cplusplus)
}
#endif
