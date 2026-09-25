// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tidal_vehicle_interfaces:msg/SafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/safety_status.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__TRAITS_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tidal_vehicle_interfaces/msg/detail/safety_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tidal_vehicle_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SafetyStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: return_required
  {
    out << "return_required: ";
    rosidl_generator_traits::value_to_yaml(msg.return_required, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SafetyStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: return_required
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "return_required: ";
    rosidl_generator_traits::value_to_yaml(msg.return_required, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SafetyStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tidal_vehicle_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use tidal_vehicle_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tidal_vehicle_interfaces::msg::SafetyStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  tidal_vehicle_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tidal_vehicle_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tidal_vehicle_interfaces::msg::SafetyStatus & msg)
{
  return tidal_vehicle_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tidal_vehicle_interfaces::msg::SafetyStatus>()
{
  return "tidal_vehicle_interfaces::msg::SafetyStatus";
}

template<>
inline const char * name<tidal_vehicle_interfaces::msg::SafetyStatus>()
{
  return "tidal_vehicle_interfaces/msg/SafetyStatus";
}

template<>
struct has_fixed_size<tidal_vehicle_interfaces::msg::SafetyStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tidal_vehicle_interfaces::msg::SafetyStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tidal_vehicle_interfaces::msg::SafetyStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__SAFETY_STATUS__TRAITS_HPP_
