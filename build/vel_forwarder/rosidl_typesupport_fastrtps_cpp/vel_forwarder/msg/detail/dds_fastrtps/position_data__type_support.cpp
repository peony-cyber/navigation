// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/position_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vel_forwarder/msg/detail/position_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vel_forwarder
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
cdr_serialize(
  const vel_forwarder::msg::PositionData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: friendcarid
  cdr << ros_message.friendcarid;
  // Member: friendx
  cdr << ros_message.friendx;
  // Member: friendy
  cdr << ros_message.friendy;
  // Member: enemycarid
  cdr << ros_message.enemycarid;
  // Member: enemyx
  cdr << ros_message.enemyx;
  // Member: enemyy
  cdr << ros_message.enemyy;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vel_forwarder::msg::PositionData & ros_message)
{
  // Member: friendcarid
  cdr >> ros_message.friendcarid;

  // Member: friendx
  cdr >> ros_message.friendx;

  // Member: friendy
  cdr >> ros_message.friendy;

  // Member: enemycarid
  cdr >> ros_message.enemycarid;

  // Member: enemyx
  cdr >> ros_message.enemyx;

  // Member: enemyy
  cdr >> ros_message.enemyy;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
get_serialized_size(
  const vel_forwarder::msg::PositionData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: friendcarid
  {
    size_t item_size = sizeof(ros_message.friendcarid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: friendx
  {
    size_t item_size = sizeof(ros_message.friendx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: friendy
  {
    size_t item_size = sizeof(ros_message.friendy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: enemycarid
  {
    size_t item_size = sizeof(ros_message.enemycarid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: enemyx
  {
    size_t item_size = sizeof(ros_message.enemyx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: enemyy
  {
    size_t item_size = sizeof(ros_message.enemyy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
max_serialized_size_PositionData(
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


  // Member: friendcarid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: friendx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: friendy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: enemycarid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: enemyx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: enemyy
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
    using DataType = vel_forwarder::msg::PositionData;
    is_plain =
      (
      offsetof(DataType, enemyy) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PositionData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vel_forwarder::msg::PositionData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PositionData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vel_forwarder::msg::PositionData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PositionData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vel_forwarder::msg::PositionData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PositionData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PositionData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PositionData__callbacks = {
  "vel_forwarder::msg",
  "PositionData",
  _PositionData__cdr_serialize,
  _PositionData__cdr_deserialize,
  _PositionData__get_serialized_size,
  _PositionData__max_serialized_size
};

static rosidl_message_type_support_t _PositionData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PositionData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vel_forwarder

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vel_forwarder
const rosidl_message_type_support_t *
get_message_type_support_handle<vel_forwarder::msg::PositionData>()
{
  return &vel_forwarder::msg::typesupport_fastrtps_cpp::_PositionData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vel_forwarder, msg, PositionData)() {
  return &vel_forwarder::msg::typesupport_fastrtps_cpp::_PositionData__handle;
}

#ifdef __cplusplus
}
#endif
