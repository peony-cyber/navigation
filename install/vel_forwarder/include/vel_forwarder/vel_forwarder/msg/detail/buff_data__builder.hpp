// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/BuffData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/buff_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_BuffData_remainingenergy
{
public:
  explicit Init_BuffData_remainingenergy(::vel_forwarder::msg::BuffData & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::BuffData remainingenergy(::vel_forwarder::msg::BuffData::_remainingenergy_type arg)
  {
    msg_.remainingenergy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::BuffData msg_;
};

class Init_BuffData_attackbuff
{
public:
  explicit Init_BuffData_attackbuff(::vel_forwarder::msg::BuffData & msg)
  : msg_(msg)
  {}
  Init_BuffData_remainingenergy attackbuff(::vel_forwarder::msg::BuffData::_attackbuff_type arg)
  {
    msg_.attackbuff = std::move(arg);
    return Init_BuffData_remainingenergy(msg_);
  }

private:
  ::vel_forwarder::msg::BuffData msg_;
};

class Init_BuffData_vulnerabilitybuff
{
public:
  explicit Init_BuffData_vulnerabilitybuff(::vel_forwarder::msg::BuffData & msg)
  : msg_(msg)
  {}
  Init_BuffData_attackbuff vulnerabilitybuff(::vel_forwarder::msg::BuffData::_vulnerabilitybuff_type arg)
  {
    msg_.vulnerabilitybuff = std::move(arg);
    return Init_BuffData_attackbuff(msg_);
  }

private:
  ::vel_forwarder::msg::BuffData msg_;
};

class Init_BuffData_defencebuff
{
public:
  explicit Init_BuffData_defencebuff(::vel_forwarder::msg::BuffData & msg)
  : msg_(msg)
  {}
  Init_BuffData_vulnerabilitybuff defencebuff(::vel_forwarder::msg::BuffData::_defencebuff_type arg)
  {
    msg_.defencebuff = std::move(arg);
    return Init_BuffData_vulnerabilitybuff(msg_);
  }

private:
  ::vel_forwarder::msg::BuffData msg_;
};

class Init_BuffData_coolingbuff
{
public:
  explicit Init_BuffData_coolingbuff(::vel_forwarder::msg::BuffData & msg)
  : msg_(msg)
  {}
  Init_BuffData_defencebuff coolingbuff(::vel_forwarder::msg::BuffData::_coolingbuff_type arg)
  {
    msg_.coolingbuff = std::move(arg);
    return Init_BuffData_defencebuff(msg_);
  }

private:
  ::vel_forwarder::msg::BuffData msg_;
};

class Init_BuffData_recoverybuff
{
public:
  Init_BuffData_recoverybuff()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BuffData_coolingbuff recoverybuff(::vel_forwarder::msg::BuffData::_recoverybuff_type arg)
  {
    msg_.recoverybuff = std::move(arg);
    return Init_BuffData_coolingbuff(msg_);
  }

private:
  ::vel_forwarder::msg::BuffData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::BuffData>()
{
  return vel_forwarder::msg::builder::Init_BuffData_recoverybuff();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__BUILDER_HPP_
