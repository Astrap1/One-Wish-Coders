// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/terrain_state.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__BUILDER_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tidal_vehicle_interfaces/msg/detail/terrain_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tidal_vehicle_interfaces
{

namespace msg
{

namespace builder
{

class Init_TerrainState_corridor_traversable
{
public:
  explicit Init_TerrainState_corridor_traversable(::tidal_vehicle_interfaces::msg::TerrainState & msg)
  : msg_(msg)
  {}
  ::tidal_vehicle_interfaces::msg::TerrainState corridor_traversable(::tidal_vehicle_interfaces::msg::TerrainState::_corridor_traversable_type arg)
  {
    msg_.corridor_traversable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

class Init_TerrainState_seconds_until_corridor_unsafe
{
public:
  explicit Init_TerrainState_seconds_until_corridor_unsafe(::tidal_vehicle_interfaces::msg::TerrainState & msg)
  : msg_(msg)
  {}
  Init_TerrainState_corridor_traversable seconds_until_corridor_unsafe(::tidal_vehicle_interfaces::msg::TerrainState::_seconds_until_corridor_unsafe_type arg)
  {
    msg_.seconds_until_corridor_unsafe = std::move(arg);
    return Init_TerrainState_corridor_traversable(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

class Init_TerrainState_tide_rate_m_per_minute
{
public:
  explicit Init_TerrainState_tide_rate_m_per_minute(::tidal_vehicle_interfaces::msg::TerrainState & msg)
  : msg_(msg)
  {}
  Init_TerrainState_seconds_until_corridor_unsafe tide_rate_m_per_minute(::tidal_vehicle_interfaces::msg::TerrainState::_tide_rate_m_per_minute_type arg)
  {
    msg_.tide_rate_m_per_minute = std::move(arg);
    return Init_TerrainState_seconds_until_corridor_unsafe(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

class Init_TerrainState_water_level_m
{
public:
  explicit Init_TerrainState_water_level_m(::tidal_vehicle_interfaces::msg::TerrainState & msg)
  : msg_(msg)
  {}
  Init_TerrainState_tide_rate_m_per_minute water_level_m(::tidal_vehicle_interfaces::msg::TerrainState::_water_level_m_type arg)
  {
    msg_.water_level_m = std::move(arg);
    return Init_TerrainState_tide_rate_m_per_minute(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

class Init_TerrainState_tide_risk
{
public:
  explicit Init_TerrainState_tide_risk(::tidal_vehicle_interfaces::msg::TerrainState & msg)
  : msg_(msg)
  {}
  Init_TerrainState_water_level_m tide_risk(::tidal_vehicle_interfaces::msg::TerrainState::_tide_risk_type arg)
  {
    msg_.tide_risk = std::move(arg);
    return Init_TerrainState_water_level_m(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

class Init_TerrainState_tide_state
{
public:
  explicit Init_TerrainState_tide_state(::tidal_vehicle_interfaces::msg::TerrainState & msg)
  : msg_(msg)
  {}
  Init_TerrainState_tide_risk tide_state(::tidal_vehicle_interfaces::msg::TerrainState::_tide_state_type arg)
  {
    msg_.tide_state = std::move(arg);
    return Init_TerrainState_tide_risk(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

class Init_TerrainState_header
{
public:
  Init_TerrainState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TerrainState_tide_state header(::tidal_vehicle_interfaces::msg::TerrainState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TerrainState_tide_state(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::TerrainState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tidal_vehicle_interfaces::msg::TerrainState>()
{
  return tidal_vehicle_interfaces::msg::builder::Init_TerrainState_header();
}

}  // namespace tidal_vehicle_interfaces

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__BUILDER_HPP_
