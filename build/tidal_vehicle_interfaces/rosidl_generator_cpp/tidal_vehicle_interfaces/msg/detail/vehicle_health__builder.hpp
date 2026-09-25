// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/vehicle_health.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__BUILDER_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tidal_vehicle_interfaces
{

namespace msg
{

namespace builder
{

class Init_VehicleHealth_fault
{
public:
  explicit Init_VehicleHealth_fault(::tidal_vehicle_interfaces::msg::VehicleHealth & msg)
  : msg_(msg)
  {}
  ::tidal_vehicle_interfaces::msg::VehicleHealth fault(::tidal_vehicle_interfaces::msg::VehicleHealth::_fault_type arg)
  {
    msg_.fault = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

class Init_VehicleHealth_payload_secured
{
public:
  explicit Init_VehicleHealth_payload_secured(::tidal_vehicle_interfaces::msg::VehicleHealth & msg)
  : msg_(msg)
  {}
  Init_VehicleHealth_fault payload_secured(::tidal_vehicle_interfaces::msg::VehicleHealth::_payload_secured_type arg)
  {
    msg_.payload_secured = std::move(arg);
    return Init_VehicleHealth_fault(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

class Init_VehicleHealth_link_ok
{
public:
  explicit Init_VehicleHealth_link_ok(::tidal_vehicle_interfaces::msg::VehicleHealth & msg)
  : msg_(msg)
  {}
  Init_VehicleHealth_payload_secured link_ok(::tidal_vehicle_interfaces::msg::VehicleHealth::_link_ok_type arg)
  {
    msg_.link_ok = std::move(arg);
    return Init_VehicleHealth_payload_secured(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

class Init_VehicleHealth_mobility_health_percent
{
public:
  explicit Init_VehicleHealth_mobility_health_percent(::tidal_vehicle_interfaces::msg::VehicleHealth & msg)
  : msg_(msg)
  {}
  Init_VehicleHealth_link_ok mobility_health_percent(::tidal_vehicle_interfaces::msg::VehicleHealth::_mobility_health_percent_type arg)
  {
    msg_.mobility_health_percent = std::move(arg);
    return Init_VehicleHealth_link_ok(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

class Init_VehicleHealth_return_reserve_percent
{
public:
  explicit Init_VehicleHealth_return_reserve_percent(::tidal_vehicle_interfaces::msg::VehicleHealth & msg)
  : msg_(msg)
  {}
  Init_VehicleHealth_mobility_health_percent return_reserve_percent(::tidal_vehicle_interfaces::msg::VehicleHealth::_return_reserve_percent_type arg)
  {
    msg_.return_reserve_percent = std::move(arg);
    return Init_VehicleHealth_mobility_health_percent(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

class Init_VehicleHealth_battery_percent
{
public:
  explicit Init_VehicleHealth_battery_percent(::tidal_vehicle_interfaces::msg::VehicleHealth & msg)
  : msg_(msg)
  {}
  Init_VehicleHealth_return_reserve_percent battery_percent(::tidal_vehicle_interfaces::msg::VehicleHealth::_battery_percent_type arg)
  {
    msg_.battery_percent = std::move(arg);
    return Init_VehicleHealth_return_reserve_percent(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

class Init_VehicleHealth_header
{
public:
  Init_VehicleHealth_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleHealth_battery_percent header(::tidal_vehicle_interfaces::msg::VehicleHealth::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VehicleHealth_battery_percent(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::VehicleHealth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tidal_vehicle_interfaces::msg::VehicleHealth>()
{
  return tidal_vehicle_interfaces::msg::builder::Init_VehicleHealth_header();
}

}  // namespace tidal_vehicle_interfaces

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__BUILDER_HPP_
