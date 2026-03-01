// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vel_forwarder:msg/BuffData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__TRAITS_HPP_
#define VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vel_forwarder/msg/detail/buff_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vel_forwarder
{

namespace msg
{

inline void to_flow_style_yaml(
  const BuffData & msg,
  std::ostream & out)
{
  out << "{";
  // member: recoverybuff
  {
    out << "recoverybuff: ";
    rosidl_generator_traits::value_to_yaml(msg.recoverybuff, out);
    out << ", ";
  }

  // member: coolingbuff
  {
    out << "coolingbuff: ";
    rosidl_generator_traits::value_to_yaml(msg.coolingbuff, out);
    out << ", ";
  }

  // member: defencebuff
  {
    out << "defencebuff: ";
    rosidl_generator_traits::value_to_yaml(msg.defencebuff, out);
    out << ", ";
  }

  // member: vulnerabilitybuff
  {
    out << "vulnerabilitybuff: ";
    rosidl_generator_traits::value_to_yaml(msg.vulnerabilitybuff, out);
    out << ", ";
  }

  // member: attackbuff
  {
    out << "attackbuff: ";
    rosidl_generator_traits::value_to_yaml(msg.attackbuff, out);
    out << ", ";
  }

  // member: remainingenergy
  {
    out << "remainingenergy: ";
    rosidl_generator_traits::value_to_yaml(msg.remainingenergy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BuffData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: recoverybuff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recoverybuff: ";
    rosidl_generator_traits::value_to_yaml(msg.recoverybuff, out);
    out << "\n";
  }

  // member: coolingbuff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coolingbuff: ";
    rosidl_generator_traits::value_to_yaml(msg.coolingbuff, out);
    out << "\n";
  }

  // member: defencebuff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "defencebuff: ";
    rosidl_generator_traits::value_to_yaml(msg.defencebuff, out);
    out << "\n";
  }

  // member: vulnerabilitybuff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vulnerabilitybuff: ";
    rosidl_generator_traits::value_to_yaml(msg.vulnerabilitybuff, out);
    out << "\n";
  }

  // member: attackbuff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attackbuff: ";
    rosidl_generator_traits::value_to_yaml(msg.attackbuff, out);
    out << "\n";
  }

  // member: remainingenergy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remainingenergy: ";
    rosidl_generator_traits::value_to_yaml(msg.remainingenergy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BuffData & msg, bool use_flow_style = false)
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
  const vel_forwarder::msg::BuffData & msg,
  std::ostream & out, size_t indentation = 0)
{
  vel_forwarder::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vel_forwarder::msg::to_yaml() instead")]]
inline std::string to_yaml(const vel_forwarder::msg::BuffData & msg)
{
  return vel_forwarder::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vel_forwarder::msg::BuffData>()
{
  return "vel_forwarder::msg::BuffData";
}

template<>
inline const char * name<vel_forwarder::msg::BuffData>()
{
  return "vel_forwarder/msg/BuffData";
}

template<>
struct has_fixed_size<vel_forwarder::msg::BuffData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vel_forwarder::msg::BuffData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vel_forwarder::msg::BuffData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__TRAITS_HPP_
