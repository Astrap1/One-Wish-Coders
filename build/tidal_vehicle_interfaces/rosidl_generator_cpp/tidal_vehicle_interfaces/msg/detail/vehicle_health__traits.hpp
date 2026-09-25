// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/vehicle_health.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__TRAITS_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tidal_vehicle_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleHealth & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: battery_percent
  {
    out << "battery_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percent, out);
    out << ", ";
  }

  // member: return_reserve_percent
  {
    out << "return_reserve_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.return_reserve_percent, out);
    out << ", ";
  }

  // member: mobility_health_percent
  {
    out << "mobility_health_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.mobility_health_percent, out);
    out << ", ";
  }

  // member: link_ok
  {
    out << "link_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.link_ok, out);
    out << ", ";
  }

  // member: payload_secured
  {
    out << "payload_secured: ";
    rosidl_generator_traits::value_to_yaml(msg.payload_secured, out);
    out << ", ";
  }

  // member: fault
  {
    out << "fault: ";
    rosidl_generator_traits::value_to_yaml(msg.fault, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleHealth & msg,
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

  // member: battery_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percent, out);
    out << "\n";
  }

  // member: return_reserve_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "return_reserve_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.return_reserve_percent, out);
    out << "\n";
  }

  // member: mobility_health_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mobility_health_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.mobility_health_percent, out);
    out << "\n";
  }

  // member: link_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "link_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.link_ok, out);
    out << "\n";
  }

  // member: payload_secured
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "payload_secured: ";
    rosidl_generator_traits::value_to_yaml(msg.payload_secured, out);
    out << "\n";
  }

  // member: fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault: ";
    rosidl_generator_traits::value_to_yaml(msg.fault, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleHealth & msg, bool use_flow_style = false)
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
  const tidal_vehicle_interfaces::msg::VehicleHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  tidal_vehicle_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tidal_vehicle_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tidal_vehicle_interfaces::msg::VehicleHealth & msg)
{
  return tidal_vehicle_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tidal_vehicle_interfaces::msg::VehicleHealth>()
{
  return "tidal_vehicle_interfaces::msg::VehicleHealth";
}

template<>
inline const char * name<tidal_vehicle_interfaces::msg::VehicleHealth>()
{
  return "tidal_vehicle_interfaces/msg/VehicleHealth";
}

template<>
struct has_fixed_size<tidal_vehicle_interfaces::msg::VehicleHealth>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tidal_vehicle_interfaces::msg::VehicleHealth>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tidal_vehicle_interfaces::msg::VehicleHealth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__VEHICLE_HEALTH__TRAITS_HPP_
