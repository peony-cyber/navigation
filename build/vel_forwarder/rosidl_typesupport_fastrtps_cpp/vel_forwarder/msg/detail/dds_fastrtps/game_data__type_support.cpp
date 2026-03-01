// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/game_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vel_forwarder/msg/detail/game_data__struct.hpp"

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
  const vel_forwarder::msg::GameData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: gamecode
  cdr << ros_message.gamecode;
  // Member: ammoleft
  cdr << ros_message.ammoleft;
  // Member: timeleft
  cdr << ros_message.timeleft;
  // Member: selfhealth
  cdr << ros_message.selfhealth;
  // Member: exteventdata
  cdr << ros_message.exteventdata;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vel_forwarder::msg::GameData & ros_message)
{
  // Member: gamecode
  cdr >> ros_message.gamecode;

  // Member: ammoleft
  cdr >> ros_message.ammoleft;

  // Member: timeleft
  cdr >> ros_message.timeleft;

  // Member: selfhealth
  cdr >> ros_message.selfhealth;

  // Member: exteventdata
  cdr >> ros_message.exteventdata;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
get_serialized_size(
  const vel_forwarder::msg::GameData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: gamecode
  {
    size_t item_size = sizeof(ros_message.gamecode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ammoleft
  {
    size_t item_size = sizeof(ros_message.ammoleft);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timeleft
  {
    size_t item_size = sizeof(ros_message.timeleft);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: selfhealth
  {
    size_t item_size = sizeof(ros_message.selfhealth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: exteventdata
  {
    size_t item_size = sizeof(ros_message.exteventdata);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vel_forwarder
max_serialized_size_GameData(
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


  // Member: gamecode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: ammoleft
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: timeleft
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: selfhealth
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: exteventdata
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vel_forwarder::msg::GameData;
    is_plain =
      (
      offsetof(DataType, exteventdata) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GameData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vel_forwarder::msg::GameData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GameData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vel_forwarder::msg::GameData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GameData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vel_forwarder::msg::GameData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GameData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GameData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GameData__callbacks = {
  "vel_forwarder::msg",
  "GameData",
  _GameData__cdr_serialize,
  _GameData__cdr_deserialize,
  _GameData__get_serialized_size,
  _GameData__max_serialized_size
};

static rosidl_message_type_support_t _GameData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GameData__callbacks,
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
get_message_type_support_handle<vel_forwarder::msg::GameData>()
{
  return &vel_forwarder::msg::typesupport_fastrtps_cpp::_GameData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vel_forwarder, msg, GameData)() {
  return &vel_forwarder::msg::typesupport_fastrtps_cpp::_GameData__handle;
}

#ifdef __cplusplus
}
#endif
