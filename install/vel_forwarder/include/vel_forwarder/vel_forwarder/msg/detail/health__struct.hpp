// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vel_forwarder:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__HEALTH__STRUCT_HPP_
#define VEL_FORWARDER__MSG__DETAIL__HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vel_forwarder__msg__Health __attribute__((deprecated))
#else
# define DEPRECATED__vel_forwarder__msg__Health __declspec(deprecated)
#endif

namespace vel_forwarder
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Health_
{
  using Type = Health_<ContainerAllocator>;

  explicit Health_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hero = 0;
      this->engineer = 0;
      this->infantry1 = 0;
      this->infantry2 = 0;
      this->reserve = 0;
      this->sentry = 0;
    }
  }

  explicit Health_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hero = 0;
      this->engineer = 0;
      this->infantry1 = 0;
      this->infantry2 = 0;
      this->reserve = 0;
      this->sentry = 0;
    }
  }

  // field types and members
  using _hero_type =
    uint16_t;
  _hero_type hero;
  using _engineer_type =
    uint16_t;
  _engineer_type engineer;
  using _infantry1_type =
    uint16_t;
  _infantry1_type infantry1;
  using _infantry2_type =
    uint16_t;
  _infantry2_type infantry2;
  using _reserve_type =
    uint16_t;
  _reserve_type reserve;
  using _sentry_type =
    uint16_t;
  _sentry_type sentry;

  // setters for named parameter idiom
  Type & set__hero(
    const uint16_t & _arg)
  {
    this->hero = _arg;
    return *this;
  }
  Type & set__engineer(
    const uint16_t & _arg)
  {
    this->engineer = _arg;
    return *this;
  }
  Type & set__infantry1(
    const uint16_t & _arg)
  {
    this->infantry1 = _arg;
    return *this;
  }
  Type & set__infantry2(
    const uint16_t & _arg)
  {
    this->infantry2 = _arg;
    return *this;
  }
  Type & set__reserve(
    const uint16_t & _arg)
  {
    this->reserve = _arg;
    return *this;
  }
  Type & set__sentry(
    const uint16_t & _arg)
  {
    this->sentry = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vel_forwarder::msg::Health_<ContainerAllocator> *;
  using ConstRawPtr =
    const vel_forwarder::msg::Health_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vel_forwarder::msg::Health_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vel_forwarder::msg::Health_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::Health_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::Health_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vel_forwarder::msg::Health_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vel_forwarder::msg::Health_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vel_forwarder::msg::Health_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vel_forwarder::msg::Health_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vel_forwarder__msg__Health
    std::shared_ptr<vel_forwarder::msg::Health_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vel_forwarder__msg__Health
    std::shared_ptr<vel_forwarder::msg::Health_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Health_ & other) const
  {
    if (this->hero != other.hero) {
      return false;
    }
    if (this->engineer != other.engineer) {
      return false;
    }
    if (this->infantry1 != other.infantry1) {
      return false;
    }
    if (this->infantry2 != other.infantry2) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    if (this->sentry != other.sentry) {
      return false;
    }
    return true;
  }
  bool operator!=(const Health_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Health_

// alias to use template instance with default allocator
using Health =
  vel_forwarder::msg::Health_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vel_forwarder

#endif  // VEL_FORWARDER__MSG__DETAIL__HEALTH__STRUCT_HPP_
