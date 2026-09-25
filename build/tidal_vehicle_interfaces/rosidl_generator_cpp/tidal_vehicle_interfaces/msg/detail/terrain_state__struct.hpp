// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/terrain_state.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__STRUCT_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__STRUCT_HPP_

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
# define DEPRECATED__tidal_vehicle_interfaces__msg__TerrainState __attribute__((deprecated))
#else
# define DEPRECATED__tidal_vehicle_interfaces__msg__TerrainState __declspec(deprecated)
#endif

namespace tidal_vehicle_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TerrainState_
{
  using Type = TerrainState_<ContainerAllocator>;

  explicit TerrainState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tide_state = "";
      this->tide_risk = 0.0f;
      this->water_level_m = 0.0f;
      this->tide_rate_m_per_minute = 0.0f;
      this->seconds_until_corridor_unsafe = 0.0f;
      this->corridor_traversable = false;
    }
  }

  explicit TerrainState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    tide_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tide_state = "";
      this->tide_risk = 0.0f;
      this->water_level_m = 0.0f;
      this->tide_rate_m_per_minute = 0.0f;
      this->seconds_until_corridor_unsafe = 0.0f;
      this->corridor_traversable = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _tide_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tide_state_type tide_state;
  using _tide_risk_type =
    float;
  _tide_risk_type tide_risk;
  using _water_level_m_type =
    float;
  _water_level_m_type water_level_m;
  using _tide_rate_m_per_minute_type =
    float;
  _tide_rate_m_per_minute_type tide_rate_m_per_minute;
  using _seconds_until_corridor_unsafe_type =
    float;
  _seconds_until_corridor_unsafe_type seconds_until_corridor_unsafe;
  using _corridor_traversable_type =
    bool;
  _corridor_traversable_type corridor_traversable;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__tide_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tide_state = _arg;
    return *this;
  }
  Type & set__tide_risk(
    const float & _arg)
  {
    this->tide_risk = _arg;
    return *this;
  }
  Type & set__water_level_m(
    const float & _arg)
  {
    this->water_level_m = _arg;
    return *this;
  }
  Type & set__tide_rate_m_per_minute(
    const float & _arg)
  {
    this->tide_rate_m_per_minute = _arg;
    return *this;
  }
  Type & set__seconds_until_corridor_unsafe(
    const float & _arg)
  {
    this->seconds_until_corridor_unsafe = _arg;
    return *this;
  }
  Type & set__corridor_traversable(
    const bool & _arg)
  {
    this->corridor_traversable = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator> *;
  using ConstRawPtr =
    const tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tidal_vehicle_interfaces__msg__TerrainState
    std::shared_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tidal_vehicle_interfaces__msg__TerrainState
    std::shared_ptr<tidal_vehicle_interfaces::msg::TerrainState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TerrainState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->tide_state != other.tide_state) {
      return false;
    }
    if (this->tide_risk != other.tide_risk) {
      return false;
    }
    if (this->water_level_m != other.water_level_m) {
      return false;
    }
    if (this->tide_rate_m_per_minute != other.tide_rate_m_per_minute) {
      return false;
    }
    if (this->seconds_until_corridor_unsafe != other.seconds_until_corridor_unsafe) {
      return false;
    }
    if (this->corridor_traversable != other.corridor_traversable) {
      return false;
    }
    return true;
  }
  bool operator!=(const TerrainState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TerrainState_

// alias to use template instance with default allocator
using TerrainState =
  tidal_vehicle_interfaces::msg::TerrainState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tidal_vehicle_interfaces

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__STRUCT_HPP_
