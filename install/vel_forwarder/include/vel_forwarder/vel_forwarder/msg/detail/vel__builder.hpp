// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/Vel.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__VEL__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/vel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_Vel_y
{
public:
  explicit Init_Vel_y(::vel_forwarder::msg::Vel & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::Vel y(::vel_forwarder::msg::Vel::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::Vel msg_;
};

class Init_Vel_x
{
public:
  Init_Vel_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vel_y x(::vel_forwarder::msg::Vel::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Vel_y(msg_);
  }

private:
  ::vel_forwarder::msg::Vel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::Vel>()
{
  return vel_forwarder::msg::builder::Init_Vel_x();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__VEL__BUILDER_HPP_
