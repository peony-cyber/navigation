// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vel_forwarder:msg/GimbalAngles.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__GIMBAL_ANGLES__BUILDER_HPP_
#define VEL_FORWARDER__MSG__DETAIL__GIMBAL_ANGLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vel_forwarder/msg/detail/gimbal_angles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vel_forwarder
{

namespace msg
{

namespace builder
{

class Init_GimbalAngles_header
{
public:
  explicit Init_GimbalAngles_header(::vel_forwarder::msg::GimbalAngles & msg)
  : msg_(msg)
  {}
  ::vel_forwarder::msg::GimbalAngles header(::vel_forwarder::msg::GimbalAngles::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vel_forwarder::msg::GimbalAngles msg_;
};

class Init_GimbalAngles_pitch
{
public:
  explicit Init_GimbalAngles_pitch(::vel_forwarder::msg::GimbalAngles & msg)
  : msg_(msg)
  {}
  Init_GimbalAngles_header pitch(::vel_forwarder::msg::GimbalAngles::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GimbalAngles_header(msg_);
  }

private:
  ::vel_forwarder::msg::GimbalAngles msg_;
};

class Init_GimbalAngles_yaw
{
public:
  Init_GimbalAngles_yaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalAngles_pitch yaw(::vel_forwarder::msg::GimbalAngles::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GimbalAngles_pitch(msg_);
  }

private:
  ::vel_forwarder::msg::GimbalAngles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vel_forwarder::msg::GimbalAngles>()
{
  return vel_forwarder::msg::builder::Init_GimbalAngles_yaw();
}

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__GIMBAL_ANGLES__BUILDER_HPP_
