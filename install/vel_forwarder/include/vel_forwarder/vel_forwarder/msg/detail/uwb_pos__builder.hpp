// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/UWBPos.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__UWB_POS__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__UWB_POS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/uwb_pos__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_UWBPos_y
{
public:
  explicit Init_UWBPos_y(::vel_forwarder::msg::UWBPos & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::UWBPos y(::vel_forwarder::msg::UWBPos::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::UWBPos msg_;
};

class Init_UWBPos_x
{
public:
  Init_UWBPos_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UWBPos_y x(::vel_forwarder::msg::UWBPos::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_UWBPos_y(msg_);
  }

private:
  ::vel_forwarder::msg::UWBPos msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::UWBPos>()
{
  return vel_forwarder::msg::builder::Init_UWBPos_x();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__UWB_POS__BUILDER_HPP_
