// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vel_forwarder:msg/BuffData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__STRUCT_HPP_
#define VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vel_forwarder__msg__BuffData __attribute__((deprecated))
#else
# define DEPRECATED__vel_forwarder__msg__BuffData __declspec(deprecated)
#endif

namespace vel_forwarder
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BuffData_
{
  using Type = BuffData_<ContainerAllocator>;

  explicit BuffData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->recoverybuff = 0;
      this->coolingbuff = 0;
      this->defencebuff = 0;
      this->vulnerabilitybuff = 0;
      this->attackbuff = 0;
      this->remainingenergy = 0;
    }
  }

  explicit BuffData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->recoverybuff = 0;
      this->coolingbuff = 0;
      this->defencebuff = 0;
      this->vulnerabilitybuff = 0;
      this->attackbuff = 0;
      this->remainingenergy = 0;
    }
  }

  // field types and members
  using _recoverybuff_type =
    uint8_t;
  _recoverybuff_type recoverybuff;
  using _coolingbuff_type =
    uint8_t;
  _coolingbuff_type coolingbuff;
  using _defencebuff_type =
    uint8_t;
  _defencebuff_type defencebuff;
  using _vulnerabilitybuff_type =
    uint8_t;
  _vulnerabilitybuff_type vulnerabilitybuff;
  using _attackbuff_type =
    uint16_t;
  _attackbuff_type attackbuff;
  using _remainingenergy_type =
    uint8_t;
  _remainingenergy_type remainingenergy;

  // setters for named parameter idiom
  Type & set__recoverybuff(
    const uint8_t & _arg)
  {
    this->recoverybuff = _arg;
    return *this;
  }
  Type & set__coolingbuff(
    const uint8_t & _arg)
  {
    this->coolingbuff = _arg;
    return *this;
  }
  Type & set__defencebuff(
    const uint8_t & _arg)
  {
    this->defencebuff = _arg;
    return *this;
  }
  Type & set__vulnerabilitybuff(
    const uint8_t & _arg)
  {
    this->vulnerabilitybuff = _arg;
    return *this;
  }
  Type & set__attackbuff(
    const uint16_t & _arg)
  {
    this->attackbuff = _arg;
    return *this;
  }
  Type & set__remainingenergy(
    const uint8_t & _arg)
  {
    this->remainingenergy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vel_forwarder::msg::BuffData_<ContainerAllocator> *;
  using ConstRawPtr =
    const vel_forwarder::msg::BuffData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::BuffData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::BuffData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vel_forwarder__msg__BuffData
    std::shared_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vel_forwarder__msg__BuffData
    std::shared_ptr<vel_forwarder::msg::BuffData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BuffData_ & other) const
  {
    if (this->recoverybuff != other.recoverybuff) {
      return false;
    }
    if (this->coolingbuff != other.coolingbuff) {
      return false;
    }
    if (this->defencebuff != other.defencebuff) {
      return false;
    }
    if (this->vulnerabilitybuff != other.vulnerabilitybuff) {
      return false;
    }
    if (this->attackbuff != other.attackbuff) {
      return false;
    }
    if (this->remainingenergy != other.remainingenergy) {
      return false;
    }
    return true;
  }
  bool operator!=(const BuffData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BuffData_

// alias to use template instance with default allocator
using BuffData =
  vel_forwarder::msg::BuffData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__BUFF_DATA__STRUCT_HPP_
