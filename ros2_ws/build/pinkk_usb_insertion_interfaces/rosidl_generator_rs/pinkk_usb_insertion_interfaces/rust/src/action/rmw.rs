
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_Goal() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__init(msg: *mut CartesianMove_Goal) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Goal>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Goal>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Goal>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lock_z: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lock_roll_pitch: bool,

}



impl Default for CartesianMove_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_Goal() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_Result() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Result__init(msg: *mut CartesianMove_Result) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Result>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Result>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Result>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub actual: geometry_msgs::msg::rmw::PoseStamped,

}



impl Default for CartesianMove_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_Result__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_Result where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_Result() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__init(msg: *mut CartesianMove_Feedback) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Feedback>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Feedback>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_Feedback>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub actual: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position_error_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation_error_deg: f64,

}



impl Default for CartesianMove_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__init(msg: *mut CartesianMove_FeedbackMessage) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_FeedbackMessage>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_FeedbackMessage>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_FeedbackMessage>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::CartesianMove_Feedback,

}



impl Default for CartesianMove_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage() }
  }
}




#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__init(msg: *mut CartesianMove_SendGoal_Request) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Request>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Request>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Request>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::CartesianMove_Goal,

}



impl Default for CartesianMove_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__init(msg: *mut CartesianMove_SendGoal_Response) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Response>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Response>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_SendGoal_Response>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for CartesianMove_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__init(msg: *mut CartesianMove_GetResult_Request) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Request>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Request>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Request>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for CartesianMove_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__init(msg: *mut CartesianMove_GetResult_Response) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Response>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Response>);
    fn pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<CartesianMove_GetResult_Response>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::CartesianMove_Result,

}



impl Default for CartesianMove_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CartesianMove_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CartesianMove_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/action/CartesianMove_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response() }
  }
}






#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct CartesianMove_SendGoal;

impl rosidl_runtime_rs::Service for CartesianMove_SendGoal {
    type Request = CartesianMove_SendGoal_Request;
    type Response = CartesianMove_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal() }
    }
}




#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct CartesianMove_GetResult;

impl rosidl_runtime_rs::Service for CartesianMove_GetResult {
    type Request = CartesianMove_GetResult_Request;
    type Response = CartesianMove_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult() }
    }
}


