// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vel_forwarder:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__HEALTH__TRAITS_HPP_
#define VEL_FORWARDER__MSG__DETAIL__HEALTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vel_forwarder/msg/detail/health__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vel_forwarder
{

namespace msg
{

inline void to_flow_style_yaml(
  const Health & msg,
  std::ostream & out)
{
  out << "{";
  // member: hero
  {
    out << "hero: ";
    rosidl_generator_traits::value_to_yaml(msg.hero, out);
    out << ", ";
  }

  // member: engineer
  {
    out << "engineer: ";
    rosidl_generator_traits::value_to_yaml(msg.engineer, out);
    out << ", ";
  }

  // member: infantry1
  {
    out << "infantry1: ";
    rosidl_generator_traits::value_to_yaml(msg.infantry1, out);
    out << ", ";
  }

  // member: infantry2
  {
    out << "infantry2: ";
    rosidl_generator_traits::value_to_yaml(msg.infantry2, out);
    out << ", ";
  }

  // member: reserve
  {
    out << "reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.reserve, out);
    out << ", ";
  }

  // member: sentry
  {
    out << "sentry: ";
    rosidl_generator_traits::value_to_yaml(msg.sentry, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Health & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: hero
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hero: ";
    rosidl_generator_traits::value_to_yaml(msg.hero, out);
    out << "\n";
  }

  // member: engineer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "engineer: ";
    rosidl_generator_traits::value_to_yaml(msg.engineer, out);
    out << "\n";
  }

  // member: infantry1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "infantry1: ";
    rosidl_generator_traits::value_to_yaml(msg.infantry1, out);
    out << "\n";
  }

  // member: infantry2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "infantry2: ";
    rosidl_generator_traits::value_to_yaml(msg.infantry2, out);
    out << "\n";
  }

  // member: reserve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.reserve, out);
    out << "\n";
  }

  // member: sentry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sentry: ";
    rosidl_generator_traits::value_to_yaml(msg.sentry, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Health & msg, bool use_flow_style = false)
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
  const vel_forwarder::msg::Health & msg,
  std::ostream & out, size_t indentation = 0)
{
  vel_forwarder::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vel_forwarder::msg::to_yaml() instead")]]
inline std::string to_yaml(const vel_forwarder::msg::Health & msg)
{
  return vel_forwarder::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vel_forwarder::msg::Health>()
{
  return "vel_forwarder::msg::Health";
}

template<>
inline const char * name<vel_forwarder::msg::Health>()
{
  return "vel_forwarder/msg/Health";
}

template<>
struct has_fixed_size<vel_forwarder::msg::Health>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vel_forwarder::msg::Health>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vel_forwarder::msg::Health>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEL_FORWARDER__MSG__DETAIL__HEALTH__TRAITS_HPP_
