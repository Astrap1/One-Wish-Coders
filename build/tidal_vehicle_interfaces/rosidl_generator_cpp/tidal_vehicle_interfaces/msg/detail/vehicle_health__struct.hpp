// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/vehicle_health.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__STRUCT_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tidal_vehicle_interfaces__msg__VehicleHealth __attribute__((deprecated))
#else
# define DEPRECATED__tidal_vehicle_interfaces__msg__VehicleHealth __declspec(deprecated)
#endif

namespace tidal_vehicle_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleHealth_
{
  using Type = VehicleHealth_<ContainerAllocator>;

  explicit VehicleHealth_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_percent = 0.0f;
      this->return_reserve_percent = 0.0f;
      this->mobility_health_percent = 0.0f;
      this->link_ok = false;
      this->payload_secured = false;
      this->fault = "";
    }
  }

  explicit VehicleHealth_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    fault(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_percent = 0.0f;
      this->return_reserve_percent = 0.0f;
      this->mobility_health_percent = 0.0f;
      this->link_ok = false;
      this->payload_secured = false;
      this->fault = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _battery_percent_type =
    float;
  _battery_percent_type battery_percent;
  using _return_reserve_percent_type =
    float;
  _return_reserve_percent_type return_reserve_percent;
  using _mobility_health_percent_type =
    float;
  _mobility_health_percent_type mobility_health_percent;
  using _link_ok_type =
    bool;
  _link_ok_type link_ok;
  using _payload_secured_type =
    bool;
  _payload_secured_type payload_secured;
  using _fault_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fault_type fault;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__battery_percent(
    const float & _arg)
  {
    this->battery_percent = _arg;
    return *this;
  }
  Type & set__return_reserve_percent(
    const float & _arg)
  {
    this->return_reserve_percent = _arg;
    return *this;
  }
  Type & set__mobility_health_percent(
    const float & _arg)
  {
    this->mobility_health_percent = _arg;
    return *this;
  }
  Type & set__link_ok(
    const bool & _arg)
  {
    this->link_ok = _arg;
    return *this;
  }
  Type & set__payload_secured(
    const bool & _arg)
  {
    this->payload_secured = _arg;
    return *this;
  }
  Type & set__fault(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fault = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator> *;
  using ConstRawPtr =
    const tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tidal_vehicle_interfaces__msg__VehicleHealth
    std::shared_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tidal_vehicle_interfaces__msg__VehicleHealth
    std::shared_ptr<tidal_vehicle_interfaces::msg::VehicleHealth_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleHealth_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->battery_percent != other.battery_percent) {
      return false;
    }
    if (this->return_reserve_percent != other.return_reserve_percent) {
      return false;
    }
    if (this->mobility_health_percent != other.mobility_health_percent) {
      return false;
    }
    if (this->link_ok != other.link_ok) {
      return false;
    }
    if (this->payload_secured != other.payload_secured) {
      return false;
    }
    if (this->fault != other.fault) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleHealth_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleHealth_

// alias to use template instance with default allocator
using VehicleHealth =
  tidal_vehicle_interfaces::msg::VehicleHealth_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tidal_vehicle_interfaces

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__STRUCT_HPP_
