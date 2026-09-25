#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "tidal_vehicle_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tidal_vehicle_interfaces__msg__SafetyStatus() -> *const std::ffi::c_void;
}

#[link(name = "tidal_vehicle_interfaces__rosidl_generator_c")]
extern "C" {
    fn tidal_vehicle_interfaces__msg__SafetyStatus__init(msg: *mut SafetyStatus) -> bool;
    fn tidal_vehicle_interfaces__msg__SafetyStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SafetyStatus>, size: usize) -> bool;
    fn tidal_vehicle_interfaces__msg__SafetyStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SafetyStatus>);
    fn tidal_vehicle_interfaces__msg__SafetyStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SafetyStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<SafetyStatus>) -> bool;
}

// Corresponds to tidal_vehicle_interfaces__msg__SafetyStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SafetyStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub return_required: bool,

}



impl Default for SafetyStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tidal_vehicle_interfaces__msg__SafetyStatus__init(&mut msg as *mut _) {
        panic!("Call to tidal_vehicle_interfaces__msg__SafetyStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SafetyStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__SafetyStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__SafetyStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__SafetyStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SafetyStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SafetyStatus where Self: Sized {
  const TYPE_NAME: &'static str = "tidal_vehicle_interfaces/msg/SafetyStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tidal_vehicle_interfaces__msg__SafetyStatus() }
  }
}


#[link(name = "tidal_vehicle_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tidal_vehicle_interfaces__msg__TerrainState() -> *const std::ffi::c_void;
}

#[link(name = "tidal_vehicle_interfaces__rosidl_generator_c")]
extern "C" {
    fn tidal_vehicle_interfaces__msg__TerrainState__init(msg: *mut TerrainState) -> bool;
    fn tidal_vehicle_interfaces__msg__TerrainState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TerrainState>, size: usize) -> bool;
    fn tidal_vehicle_interfaces__msg__TerrainState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TerrainState>);
    fn tidal_vehicle_interfaces__msg__TerrainState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TerrainState>, out_seq: *mut rosidl_runtime_rs::Sequence<TerrainState>) -> bool;
}

// Corresponds to tidal_vehicle_interfaces__msg__TerrainState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TerrainState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tide_state: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tidal_vehicle_interfaces__msg__TerrainState__init(&mut msg as *mut _) {
        panic!("Call to tidal_vehicle_interfaces__msg__TerrainState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TerrainState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__TerrainState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__TerrainState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__TerrainState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TerrainState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TerrainState where Self: Sized {
  const TYPE_NAME: &'static str = "tidal_vehicle_interfaces/msg/TerrainState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tidal_vehicle_interfaces__msg__TerrainState() }
  }
}


#[link(name = "tidal_vehicle_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tidal_vehicle_interfaces__msg__VehicleHealth() -> *const std::ffi::c_void;
}

#[link(name = "tidal_vehicle_interfaces__rosidl_generator_c")]
extern "C" {
    fn tidal_vehicle_interfaces__msg__VehicleHealth__init(msg: *mut VehicleHealth) -> bool;
    fn tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleHealth>, size: usize) -> bool;
    fn tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleHealth>);
    fn tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleHealth>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleHealth>) -> bool;
}

// Corresponds to tidal_vehicle_interfaces__msg__VehicleHealth
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleHealth {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


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
    pub fault: rosidl_runtime_rs::String,

}



impl Default for VehicleHealth {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tidal_vehicle_interfaces__msg__VehicleHealth__init(&mut msg as *mut _) {
        panic!("Call to tidal_vehicle_interfaces__msg__VehicleHealth__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleHealth {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleHealth {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleHealth where Self: Sized {
  const TYPE_NAME: &'static str = "tidal_vehicle_interfaces/msg/VehicleHealth";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tidal_vehicle_interfaces__msg__VehicleHealth() }
  }
}


