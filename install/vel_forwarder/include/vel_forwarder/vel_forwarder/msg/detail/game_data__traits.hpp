// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__GAME_DATA__TRAITS_HPP_
#define VEL_FORWARDER__MSG__DETAIL__GAME_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vel_forwarder/msg/detail/game_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vel_forwarder
{

namespace msg
{

inline void to_flow_style_yaml(
  const GameData & msg,
  std::ostream & out)
{
  out << "{";
  // member: gamecode
  {
    out << "gamecode: ";
    rosidl_generator_traits::value_to_yaml(msg.gamecode, out);
    out << ", ";
  }

  // member: ammoleft
  {
    out << "ammoleft: ";
    rosidl_generator_traits::value_to_yaml(msg.ammoleft, out);
    out << ", ";
  }

  // member: timeleft
  {
    out << "timeleft: ";
    rosidl_generator_traits::value_to_yaml(msg.timeleft, out);
    out << ", ";
  }

  // member: selfhealth
  {
    out << "selfhealth: ";
    rosidl_generator_traits::value_to_yaml(msg.selfhealth, out);
    out << ", ";
  }

  // member: exteventdata
  {
    out << "exteventdata: ";
    rosidl_generator_traits::value_to_yaml(msg.exteventdata, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GameData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gamecode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gamecode: ";
    rosidl_generator_traits::value_to_yaml(msg.gamecode, out);
    out << "\n";
  }

  // member: ammoleft
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ammoleft: ";
    rosidl_generator_traits::value_to_yaml(msg.ammoleft, out);
    out << "\n";
  }

  // member: timeleft
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeleft: ";
    rosidl_generator_traits::value_to_yaml(msg.timeleft, out);
    out << "\n";
  }

  // member: selfhealth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selfhealth: ";
    rosidl_generator_traits::value_to_yaml(msg.selfhealth, out);
    out << "\n";
  }

  // member: exteventdata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exteventdata: ";
    rosidl_generator_traits::value_to_yaml(msg.exteventdata, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GameData & msg, bool use_flow_style = false)
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
  const vel_forwarder::msg::GameData & msg,
  std::ostream & out, size_t indentation = 0)
{
  vel_forwarder::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vel_forwarder::msg::to_yaml() instead")]]
inline std::string to_yaml(const vel_forwarder::msg::GameData & msg)
{
  return vel_forwarder::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vel_forwarder::msg::GameData>()
{
  return "vel_forwarder::msg::GameData";
}

template<>
inline const char * name<vel_forwarder::msg::GameData>()
{
  return "vel_forwarder/msg/GameData";
}

template<>
struct has_fixed_size<vel_forwarder::msg::GameData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vel_forwarder::msg::GameData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vel_forwarder::msg::GameData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEL_FORWARDER__MSG__DETAIL__GAME_DATA__TRAITS_HPP_
