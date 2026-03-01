// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vel_forwarder:msg/Health.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/health__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vel_forwarder/msg/detail/health__struct.hpp"

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
  const vel_forwarder::msg::Health & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: hero
  cdr << ros_message.hero;
  // Member: engineer
  cdr << ros_message.engineer;
  // Member: infantry1
  cdr << ros_message.infantry1;
  // Member: infantry2
  cdr << ros_message.infantry2;
  // Member: reserve
  cdr << ros_message.reserve;
  // Member: sentry
  cdr << ros_message.sentry;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vel_forwarder::msg::Health & ros_message)
{
  // Member: hero
  cdr >> ros_message.hero;

  // Member: engineer
  cdr >> ros_message.engineer;

  // Member: infantry1
  cdr >> ros_message.infantry1;

  // Member: infantry2
  cdr >> ros_message.infantry2;

  // Member: reserve
  cdr >> ros_message.reserve;

  // Member: sentry
  cdr >> ros_message.sentry;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
get_serialized_size(
  const vel_forwarder::msg::Health & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: hero
  {
    size_t item_size = sizeof(ros_message.hero);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: engineer
  {
    size_t item_size = sizeof(ros_message.engineer);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: infantry1
  {
    size_t item_size = sizeof(ros_message.infantry1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: infantry2
  {
    size_t item_size = sizeof(ros_message.infantry2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reserve
  {
    size_t item_size = sizeof(ros_message.reserve);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sentry
  {
    size_t item_size = sizeof(ros_message.sentry);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
max_serialized_size_Health(
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


  // Member: hero
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: engineer
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: infantry1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: infantry2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: reserve
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: sentry
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
    using DataType = vel_forwarder::msg::Health;
    is_plain =
      (
      offsetof(DataType, sentry) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Health__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vel_forwarder::msg::Health *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Health__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vel_forwarder::msg::Health *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Health__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vel_forwarder::msg::Health *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Health__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Health(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Health__callbacks = {
  "vel_forwarder::msg",
  "Health",
  _Health__cdr_serialize,
  _Health__cdr_deserialize,
  _Health__get_serialized_size,
  _Health__max_serialized_size
};

static rosidl_message_type_support_t _Health__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Health__callbacks,
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
get_message_type_support_handle<vel_forwarder::msg::Health>()
{
  return &vel_forwarder::msg::typesupport_fastrtps_cpp::_Health__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vel_forwarder, msg, Health)() {
  return &vel_forwarder::msg::typesupport_fastrtps_cpp::_Health__handle;
}

#ifdef __cplusplus
}
#endif
