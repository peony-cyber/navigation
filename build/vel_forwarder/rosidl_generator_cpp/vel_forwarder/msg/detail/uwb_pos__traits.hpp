// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vel_forwarder:msg/UWBPos.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__UWB_POS__TRAITS_HPP_
#define VEL_FORWARDER__MSG__DETAIL__UWB_POS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vel_forwarder/msg/detail/uwb_pos__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vel_forwarder
{

namespace msg
{

inline void to_flow_style_yaml(
  const UWBPos & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UWBPos & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UWBPos & msg, bool use_flow_style = false)
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
  const vel_forwarder::msg::UWBPos & msg,
  std::ostream & out, size_t indentation = 0)
{
  vel_forwarder::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vel_forwarder::msg::to_yaml() instead")]]
inline std::string to_yaml(const vel_forwarder::msg::UWBPos & msg)
{
  return vel_forwarder::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vel_forwarder::msg::UWBPos>()
{
  return "vel_forwarder::msg::UWBPos";
}

template<>
inline const char * name<vel_forwarder::msg::UWBPos>()
{
  return "vel_forwarder/msg/UWBPos";
}

template<>
struct has_fixed_size<vel_forwarder::msg::UWBPos>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vel_forwarder::msg::UWBPos>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vel_forwarder::msg::UWBPos>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEL_FORWARDER__MSG__DETAIL__UWB_POS__TRAITS_HPP_
