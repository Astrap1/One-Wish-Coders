// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tidal_vehicle_interfaces/msg/terrain_state.hpp"


#ifndef TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__TRAITS_HPP_
#define TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tidal_vehicle_interfaces/msg/detail/terrain_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tidal_vehicle_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TerrainState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: tide_state
  {
    out << "tide_state: ";
    rosidl_generator_traits::value_to_yaml(msg.tide_state, out);
    out << ", ";
  }

  // member: tide_risk
  {
    out << "tide_risk: ";
    rosidl_generator_traits::value_to_yaml(msg.tide_risk, out);
    out << ", ";
  }

  // member: water_level_m
  {
    out << "water_level_m: ";
    rosidl_generator_traits::value_to_yaml(msg.water_level_m, out);
    out << ", ";
  }

  // member: tide_rate_m_per_minute
  {
    out << "tide_rate_m_per_minute: ";
    rosidl_generator_traits::value_to_yaml(msg.tide_rate_m_per_minute, out);
    out << ", ";
  }

  // member: seconds_until_corridor_unsafe
  {
    out << "seconds_until_corridor_unsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.seconds_until_corridor_unsafe, out);
    out << ", ";
  }

  // member: corridor_traversable
  {
    out << "corridor_traversable: ";
    rosidl_generator_traits::value_to_yaml(msg.corridor_traversable, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TerrainState & msg,
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

  // member: tide_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tide_state: ";
    rosidl_generator_traits::value_to_yaml(msg.tide_state, out);
    out << "\n";
  }

  // member: tide_risk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tide_risk: ";
    rosidl_generator_traits::value_to_yaml(msg.tide_risk, out);
    out << "\n";
  }

  // member: water_level_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "water_level_m: ";
    rosidl_generator_traits::value_to_yaml(msg.water_level_m, out);
    out << "\n";
  }

  // member: tide_rate_m_per_minute
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tide_rate_m_per_minute: ";
    rosidl_generator_traits::value_to_yaml(msg.tide_rate_m_per_minute, out);
    out << "\n";
  }

  // member: seconds_until_corridor_unsafe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seconds_until_corridor_unsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.seconds_until_corridor_unsafe, out);
    out << "\n";
  }

  // member: corridor_traversable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "corridor_traversable: ";
    rosidl_generator_traits::value_to_yaml(msg.corridor_traversable, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TerrainState & msg, bool use_flow_style = false)
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
  const tidal_vehicle_interfaces::msg::TerrainState & msg,
  std::ostream & out, size_t indentation = 0)
{
  tidal_vehicle_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tidal_vehicle_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tidal_vehicle_interfaces::msg::TerrainState & msg)
{
  return tidal_vehicle_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tidal_vehicle_interfaces::msg::TerrainState>()
{
  return "tidal_vehicle_interfaces::msg::TerrainState";
}

template<>
inline const char * name<tidal_vehicle_interfaces::msg::TerrainState>()
{
  return "tidal_vehicle_interfaces/msg/TerrainState";
}

template<>
struct has_fixed_size<tidal_vehicle_interfaces::msg::TerrainState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tidal_vehicle_interfaces::msg::TerrainState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tidal_vehicle_interfaces::msg::TerrainState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TIDAL_VEHICLE_INTERFACES__MSG__DETAIL__TERRAIN_STATE__TRAITS_HPP_
