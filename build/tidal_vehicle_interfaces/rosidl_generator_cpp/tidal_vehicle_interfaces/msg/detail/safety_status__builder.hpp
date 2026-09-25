// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tidal_vehicle_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/safety_status.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__BUILDER_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tidal_vehicle_interfaces/msg/detail/safety_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tidal_vehicle_interfaces
{

namespace msg
{

namespace builder
{

class Init_SafetyStatus_return_required
{
public:
  explicit Init_SafetyStatus_return_required(::tidal_vehicle_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  ::tidal_vehicle_interfaces::msg::SafetyStatus return_required(::tidal_vehicle_interfaces::msg::SafetyStatus::_return_required_type arg)
  {
    msg_.return_required = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_reason
{
public:
  explicit Init_SafetyStatus_reason(::tidal_vehicle_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_return_required reason(::tidal_vehicle_interfaces::msg::SafetyStatus::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_SafetyStatus_return_required(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_state
{
public:
  explicit Init_SafetyStatus_state(::tidal_vehicle_interfaces::msg::SafetyStatus & msg)
  : msg_(msg)
  {}
  Init_SafetyStatus_reason state(::tidal_vehicle_interfaces::msg::SafetyStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_SafetyStatus_reason(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::SafetyStatus msg_;
};

class Init_SafetyStatus_header
{
public:
  Init_SafetyStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafetyStatus_state header(::tidal_vehicle_interfaces::msg::SafetyStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SafetyStatus_state(msg_);
  }

private:
  ::tidal_vehicle_interfaces::msg::SafetyStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tidal_vehicle_interfaces::msg::SafetyStatus>()
{
  return tidal_vehicle_interfaces::msg::builder::Init_SafetyStatus_header();
}

}  // namespace tidal_vehicle_interfaces

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__BUILDER_HPP_
