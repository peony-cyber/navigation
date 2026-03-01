// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vel_forwarder:msg/PositionData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__STRUCT_HPP_
#define VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vel_forwarder__msg__PositionData __attribute__((deprecated))
#else
# define DEPRECATED__vel_forwarder__msg__PositionData __declspec(deprecated)
#endif

namespace vel_forwarder
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionData_
{
  using Type = PositionData_<ContainerAllocator>;

  explicit PositionData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->friendcarid = 0;
      this->friendx = 0;
      this->friendy = 0;
      this->enemycarid = 0;
      this->enemyx = 0;
      this->enemyy = 0;
    }
  }

  explicit PositionData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->friendcarid = 0;
      this->friendx = 0;
      this->friendy = 0;
      this->enemycarid = 0;
      this->enemyx = 0;
      this->enemyy = 0;
    }
  }

  // field types and members
  using _friendcarid_type =
    uint8_t;
  _friendcarid_type friendcarid;
  using _friendx_type =
    int16_t;
  _friendx_type friendx;
  using _friendy_type =
    int16_t;
  _friendy_type friendy;
  using _enemycarid_type =
    uint8_t;
  _enemycarid_type enemycarid;
  using _enemyx_type =
    int16_t;
  _enemyx_type enemyx;
  using _enemyy_type =
    int16_t;
  _enemyy_type enemyy;

  // setters for named parameter idiom
  Type & set__friendcarid(
    const uint8_t & _arg)
  {
    this->friendcarid = _arg;
    return *this;
  }
  Type & set__friendx(
    const int16_t & _arg)
  {
    this->friendx = _arg;
    return *this;
  }
  Type & set__friendy(
    const int16_t & _arg)
  {
    this->friendy = _arg;
    return *this;
  }
  Type & set__enemycarid(
    const uint8_t & _arg)
  {
    this->enemycarid = _arg;
    return *this;
  }
  Type & set__enemyx(
    const int16_t & _arg)
  {
    this->enemyx = _arg;
    return *this;
  }
  Type & set__enemyy(
    const int16_t & _arg)
  {
    this->enemyy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vel_forwarder::msg::PositionData_<ContainerAllocator> *;
  using ConstRawPtr =
    const vel_forwarder::msg::PositionData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::PositionData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::PositionData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vel_forwarder__msg__PositionData
    std::shared_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vel_forwarder__msg__PositionData
    std::shared_ptr<vel_forwarder::msg::PositionData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionData_ & other) const
  {
    if (this->friendcarid != other.friendcarid) {
      return false;
    }
    if (this->friendx != other.friendx) {
      return false;
    }
    if (this->friendy != other.friendy) {
      return false;
    }
    if (this->enemycarid != other.enemycarid) {
      return false;
    }
    if (this->enemyx != other.enemyx) {
      return false;
    }
    if (this->enemyy != other.enemyy) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionData_

// alias to use template instance with default allocator
using PositionData =
  vel_forwarder::msg::PositionData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__POSITION_DATA__STRUCT_HPP_
