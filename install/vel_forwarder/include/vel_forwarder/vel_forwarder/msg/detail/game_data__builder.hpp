// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__GAME_DATA__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__GAME_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/game_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_GameData_exteventdata
{
public:
  explicit Init_GameData_exteventdata(::vel_forwarder::msg::GameData & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::GameData exteventdata(::vel_forwarder::msg::GameData::_exteventdata_type arg)
  {
    msg_.exteventdata = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::GameData msg_;
};

class Init_GameData_selfhealth
{
public:
  explicit Init_GameData_selfhealth(::vel_forwarder::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_exteventdata selfhealth(::vel_forwarder::msg::GameData::_selfhealth_type arg)
  {
    msg_.selfhealth = std::move(arg);
    return Init_GameData_exteventdata(msg_);
  }

private:
  ::vel_forwarder::msg::GameData msg_;
};

class Init_GameData_timeleft
{
public:
  explicit Init_GameData_timeleft(::vel_forwarder::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_selfhealth timeleft(::vel_forwarder::msg::GameData::_timeleft_type arg)
  {
    msg_.timeleft = std::move(arg);
    return Init_GameData_selfhealth(msg_);
  }

private:
  ::vel_forwarder::msg::GameData msg_;
};

class Init_GameData_ammoleft
{
public:
  explicit Init_GameData_ammoleft(::vel_forwarder::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_timeleft ammoleft(::vel_forwarder::msg::GameData::_ammoleft_type arg)
  {
    msg_.ammoleft = std::move(arg);
    return Init_GameData_timeleft(msg_);
  }

private:
  ::vel_forwarder::msg::GameData msg_;
};

class Init_GameData_gamecode
{
public:
  Init_GameData_gamecode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameData_ammoleft gamecode(::vel_forwarder::msg::GameData::_gamecode_type arg)
  {
    msg_.gamecode = std::move(arg);
    return Init_GameData_ammoleft(msg_);
  }

private:
  ::vel_forwarder::msg::GameData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::GameData>()
{
  return vel_forwarder::msg::builder::Init_GameData_gamecode();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__GAME_DATA__BUILDER_HPP_
