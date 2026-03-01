// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/position_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_PositionData_enemyy
{
public:
  explicit Init_PositionData_enemyy(::vel_forwarder::msg::PositionData & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::PositionData enemyy(::vel_forwarder::msg::PositionData::_enemyy_type arg)
  {
    msg_.enemyy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::PositionData msg_;
};

class Init_PositionData_enemyx
{
public:
  explicit Init_PositionData_enemyx(::vel_forwarder::msg::PositionData & msg)
  : msg_(msg)
  {}
  Init_PositionData_enemyy enemyx(::vel_forwarder::msg::PositionData::_enemyx_type arg)
  {
    msg_.enemyx = std::move(arg);
    return Init_PositionData_enemyy(msg_);
  }

private:
  ::vel_forwarder::msg::PositionData msg_;
};

class Init_PositionData_enemycarid
{
public:
  explicit Init_PositionData_enemycarid(::vel_forwarder::msg::PositionData & msg)
  : msg_(msg)
  {}
  Init_PositionData_enemyx enemycarid(::vel_forwarder::msg::PositionData::_enemycarid_type arg)
  {
    msg_.enemycarid = std::move(arg);
    return Init_PositionData_enemyx(msg_);
  }

private:
  ::vel_forwarder::msg::PositionData msg_;
};

class Init_PositionData_friendy
{
public:
  explicit Init_PositionData_friendy(::vel_forwarder::msg::PositionData & msg)
  : msg_(msg)
  {}
  Init_PositionData_enemycarid friendy(::vel_forwarder::msg::PositionData::_friendy_type arg)
  {
    msg_.friendy = std::move(arg);
    return Init_PositionData_enemycarid(msg_);
  }

private:
  ::vel_forwarder::msg::PositionData msg_;
};

class Init_PositionData_friendx
{
public:
  explicit Init_PositionData_friendx(::vel_forwarder::msg::PositionData & msg)
  : msg_(msg)
  {}
  Init_PositionData_friendy friendx(::vel_forwarder::msg::PositionData::_friendx_type arg)
  {
    msg_.friendx = std::move(arg);
    return Init_PositionData_friendy(msg_);
  }

private:
  ::vel_forwarder::msg::PositionData msg_;
};

class Init_PositionData_friendcarid
{
public:
  Init_PositionData_friendcarid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionData_friendx friendcarid(::vel_forwarder::msg::PositionData::_friendcarid_type arg)
  {
    msg_.friendcarid = std::move(arg);
    return Init_PositionData_friendx(msg_);
  }

private:
  ::vel_forwarder::msg::PositionData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::PositionData>()
{
  return vel_forwarder::msg::builder::Init_PositionData_friendcarid();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__BUILDER_HPP_
