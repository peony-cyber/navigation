// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__HEALTH__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_Health_sentry
{
public:
  explicit Init_Health_sentry(::vel_forwarder::msg::Health & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::Health sentry(::vel_forwarder::msg::Health::_sentry_type arg)
  {
    msg_.sentry = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::Health msg_;
};

class Init_Health_reserve
{
public:
  explicit Init_Health_reserve(::vel_forwarder::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_sentry reserve(::vel_forwarder::msg::Health::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_Health_sentry(msg_);
  }

private:
  ::vel_forwarder::msg::Health msg_;
};

class Init_Health_infantry2
{
public:
  explicit Init_Health_infantry2(::vel_forwarder::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_reserve infantry2(::vel_forwarder::msg::Health::_infantry2_type arg)
  {
    msg_.infantry2 = std::move(arg);
    return Init_Health_reserve(msg_);
  }

private:
  ::vel_forwarder::msg::Health msg_;
};

class Init_Health_infantry1
{
public:
  explicit Init_Health_infantry1(::vel_forwarder::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_infantry2 infantry1(::vel_forwarder::msg::Health::_infantry1_type arg)
  {
    msg_.infantry1 = std::move(arg);
    return Init_Health_infantry2(msg_);
  }

private:
  ::vel_forwarder::msg::Health msg_;
};

class Init_Health_engineer
{
public:
  explicit Init_Health_engineer(::vel_forwarder::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_infantry1 engineer(::vel_forwarder::msg::Health::_engineer_type arg)
  {
    msg_.engineer = std::move(arg);
    return Init_Health_infantry1(msg_);
  }

private:
  ::vel_forwarder::msg::Health msg_;
};

class Init_Health_hero
{
public:
  Init_Health_hero()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Health_engineer hero(::vel_forwarder::msg::Health::_hero_type arg)
  {
    msg_.hero = std::move(arg);
    return Init_Health_engineer(msg_);
  }

private:
  ::vel_forwarder::msg::Health msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::Health>()
{
  return vel_forwarder::msg::builder::Init_Health_hero();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__HEALTH__BUILDER_HPP_
