// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__TRAITS_HPP_
#define VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vel_forwarder/msg/detail/position_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vel_forwarder
{

namespace msg
{

inline void to_flow_style_yaml(
  const PositionData & msg,
  std::ostream & out)
{
  out << "{";
  // member: friendcarid
  {
    out << "friendcarid: ";
    rosidl_generator_traits::value_to_yaml(msg.friendcarid, out);
    out << ", ";
  }

  // member: friendx
  {
    out << "friendx: ";
    rosidl_generator_traits::value_to_yaml(msg.friendx, out);
    out << ", ";
  }

  // member: friendy
  {
    out << "friendy: ";
    rosidl_generator_traits::value_to_yaml(msg.friendy, out);
    out << ", ";
  }

  // member: enemycarid
  {
    out << "enemycarid: ";
    rosidl_generator_traits::value_to_yaml(msg.enemycarid, out);
    out << ", ";
  }

  // member: enemyx
  {
    out << "enemyx: ";
    rosidl_generator_traits::value_to_yaml(msg.enemyx, out);
    out << ", ";
  }

  // member: enemyy
  {
    out << "enemyy: ";
    rosidl_generator_traits::value_to_yaml(msg.enemyy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PositionData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: friendcarid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "friendcarid: ";
    rosidl_generator_traits::value_to_yaml(msg.friendcarid, out);
    out << "\n";
  }

  // member: friendx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "friendx: ";
    rosidl_generator_traits::value_to_yaml(msg.friendx, out);
    out << "\n";
  }

  // member: friendy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "friendy: ";
    rosidl_generator_traits::value_to_yaml(msg.friendy, out);
    out << "\n";
  }

  // member: enemycarid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemycarid: ";
    rosidl_generator_traits::value_to_yaml(msg.enemycarid, out);
    out << "\n";
  }

  // member: enemyx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemyx: ";
    rosidl_generator_traits::value_to_yaml(msg.enemyx, out);
    out << "\n";
  }

  // member: enemyy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemyy: ";
    rosidl_generator_traits::value_to_yaml(msg.enemyy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PositionData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vel_forwarder

namespace rosidl_generator_traits
{

[[deprecated("use vel_forwarder::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vel_forwarder::msg::PositionData & msg,
  std::ostream & out, size_t indentation = 0)
{
  vel_forwarder::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vel_forwarder::msg::to_yaml() instead")]]
inline std::string to_yaml(const vel_forwarder::msg::PositionData & msg)
{
  return vel_forwarder::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vel_forwarder::msg::PositionData>()
{
  return "vel_forwarder::msg::PositionData";
}

template<>
inline const char * name<vel_forwarder::msg::PositionData>()
{
  return "vel_forwarder/msg/PositionData";
}

template<>
struct has_fixed_size<vel_forwarder::msg::PositionData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vel_forwarder::msg::PositionData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vel_forwarder::msg::PositionData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__TRAITS_HPP_
