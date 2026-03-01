// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__GAME_DATA__STRUCT_HPP_
#define VEL_FORWARDER__MSG__DETAIL__GAME_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vel_forwarder__msg__GameData __attribute__((deprecated))
#else
# define DEPRECATED__vel_forwarder__msg__GameData __declspec(deprecated)
#endif

namespace vel_forwarder
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GameData_
{
  using Type = GameData_<ContainerAllocator>;

  explicit GameData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gamecode = 0;
      this->ammoleft = 0;
      this->timeleft = 0;
      this->selfhealth = 0;
      this->exteventdata = 0ul;
    }
  }

  explicit GameData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gamecode = 0;
      this->ammoleft = 0;
      this->timeleft = 0;
      this->selfhealth = 0;
      this->exteventdata = 0ul;
    }
  }

  // field types and members
  using _gamecode_type =
    uint16_t;
  _gamecode_type gamecode;
  using _ammoleft_type =
    uint16_t;
  _ammoleft_type ammoleft;
  using _timeleft_type =
    uint16_t;
  _timeleft_type timeleft;
  using _selfhealth_type =
    uint16_t;
  _selfhealth_type selfhealth;
  using _exteventdata_type =
    uint32_t;
  _exteventdata_type exteventdata;

  // setters for named parameter idiom
  Type & set__gamecode(
    const uint16_t & _arg)
  {
    this->gamecode = _arg;
    return *this;
  }
  Type & set__ammoleft(
    const uint16_t & _arg)
  {
    this->ammoleft = _arg;
    return *this;
  }
  Type & set__timeleft(
    const uint16_t & _arg)
  {
    this->timeleft = _arg;
    return *this;
  }
  Type & set__selfhealth(
    const uint16_t & _arg)
  {
    this->selfhealth = _arg;
    return *this;
  }
  Type & set__exteventdata(
    const uint32_t & _arg)
  {
    this->exteventdata = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vel_forwarder::msg::GameData_<ContainerAllocator> *;
  using ConstRawPtr =
    const vel_forwarder::msg::GameData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vel_forwarder::msg::GameData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vel_forwarder::msg::GameData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::GameData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::GameData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::GameData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::GameData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vel_forwarder::msg::GameData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vel_forwarder::msg::GameData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vel_forwarder__msg__GameData
    std::shared_ptr<vel_forwarder::msg::GameData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vel_forwarder__msg__GameData
    std::shared_ptr<vel_forwarder::msg::GameData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GameData_ & other) const
  {
    if (this->gamecode != other.gamecode) {
      return false;
    }
    if (this->ammoleft != other.ammoleft) {
      return false;
    }
    if (this->timeleft != other.timeleft) {
      return false;
    }
    if (this->selfhealth != other.selfhealth) {
      return false;
    }
    if (this->exteventdata != other.exteventdata) {
      return false;
    }
    return true;
  }
  bool operator!=(const GameData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GameData_

// alias to use template instance with default allocator
using GameData =
  vel_forwarder::msg::GameData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__GAME_DATA__STRUCT_HPP_
