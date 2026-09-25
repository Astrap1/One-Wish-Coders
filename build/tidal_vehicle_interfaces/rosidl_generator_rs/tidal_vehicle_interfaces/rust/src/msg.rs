#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to tidal_vehicle_interfaces__msg__SafetyStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SafetyStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub return_required: bool,

}



impl Default for SafetyStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SafetyStatus::default())
  }
}

impl rosidl_runtime_rs::Message for SafetyStatus {
  type RmwMsg = super::msg::rmw::SafetyStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        state: msg.state.as_str().into(),
        reason: msg.reason.as_str().into(),
        return_required: msg.return_required,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        state: msg.state.as_str().into(),
        reason: msg.reason.as_str().into(),
      return_required: msg.return_required,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      state: msg.state.to_string(),
      reason: msg.reason.to_string(),
      return_required: msg.return_required,
    }
  }
}


// Corresponds to tidal_vehicle_interfaces__msg__TerrainState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TerrainState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tide_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tide_risk: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub water_level_m: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tide_rate_m_per_minute: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub seconds_until_corridor_unsafe: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub corridor_traversable: bool,

}



impl Default for TerrainState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TerrainState::default())
  }
}

impl rosidl_runtime_rs::Message for TerrainState {
  type RmwMsg = super::msg::rmw::TerrainState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        tide_state: msg.tide_state.as_str().into(),
        tide_risk: msg.tide_risk,
        water_level_m: msg.water_level_m,
        tide_rate_m_per_minute: msg.tide_rate_m_per_minute,
        seconds_until_corridor_unsafe: msg.seconds_until_corridor_unsafe,
        corridor_traversable: msg.corridor_traversable,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        tide_state: msg.tide_state.as_str().into(),
      tide_risk: msg.tide_risk,
      water_level_m: msg.water_level_m,
      tide_rate_m_per_minute: msg.tide_rate_m_per_minute,
      seconds_until_corridor_unsafe: msg.seconds_until_corridor_unsafe,
      corridor_traversable: msg.corridor_traversable,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      tide_state: msg.tide_state.to_string(),
      tide_risk: msg.tide_risk,
      water_level_m: msg.water_level_m,
      tide_rate_m_per_minute: msg.tide_rate_m_per_minute,
      seconds_until_corridor_unsafe: msg.seconds_until_corridor_unsafe,
      corridor_traversable: msg.corridor_traversable,
    }
  }
}


// Corresponds to tidal_vehicle_interfaces__msg__VehicleHealth

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleHealth {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub battery_percent: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub return_reserve_percent: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mobility_health_percent: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub link_ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub payload_secured: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fault: std::string::String,

}



impl Default for VehicleHealth {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleHealth::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleHealth {
  type RmwMsg = super::msg::rmw::VehicleHealth;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        battery_percent: msg.battery_percent,
        return_reserve_percent: msg.return_reserve_percent,
        mobility_health_percent: msg.mobility_health_percent,
        link_ok: msg.link_ok,
        payload_secured: msg.payload_secured,
        fault: msg.fault.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      battery_percent: msg.battery_percent,
      return_reserve_percent: msg.return_reserve_percent,
      mobility_health_percent: msg.mobility_health_percent,
      link_ok: msg.link_ok,
      payload_secured: msg.payload_secured,
        fault: msg.fault.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      battery_percent: msg.battery_percent,
      return_reserve_percent: msg.return_reserve_percent,
      mobility_health_percent: msg.mobility_health_percent,
      link_ok: msg.link_ok,
      payload_secured: msg.payload_secured,
      fault: msg.fault.to_string(),
    }
  }
}


