#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__Keypoint2D() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__msg__Keypoint2D__init(msg: *mut Keypoint2D) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Keypoint2D>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Keypoint2D>);
    fn pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Keypoint2D>, out_seq: *mut rosidl_runtime_rs::Sequence<Keypoint2D>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__msg__Keypoint2D
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// YOLO가 검출한 하나의 2D keypoint.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Keypoint2D {

    // This member is not documented.
    #[allow(missing_docs)]
    pub index: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub visible: bool,

}



impl Default for Keypoint2D {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__msg__Keypoint2D__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__msg__Keypoint2D__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Keypoint2D {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Keypoint2D {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Keypoint2D where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/msg/Keypoint2D";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__Keypoint2D() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__UsbPortDetection() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(msg: *mut UsbPortDetection) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UsbPortDetection>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UsbPortDetection>);
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UsbPortDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<UsbPortDetection>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__msg__UsbPortDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 한 USB-A 포트 후보에 대한 YOLO keypoint 결과.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UsbPortDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detection_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub class_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub object_confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bbox: vision_msgs::msg::rmw::BoundingBox2D,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_image_width: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_image_height: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub keypoints: [super::super::msg::rmw::Keypoint2D; 4],

}



impl Default for UsbPortDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UsbPortDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UsbPortDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UsbPortDetection where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/msg/UsbPortDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__UsbPortDetection() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(msg: *mut UsbPortDetectionArray) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UsbPortDetectionArray>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UsbPortDetectionArray>);
    fn pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UsbPortDetectionArray>, out_seq: *mut rosidl_runtime_rs::Sequence<UsbPortDetectionArray>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 같은 영상 프레임에서 검출된 USB-A 포트 후보 목록.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UsbPortDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: rosidl_runtime_rs::Sequence<super::super::msg::rmw::UsbPortDetection>,

}



impl Default for UsbPortDetectionArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UsbPortDetectionArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UsbPortDetectionArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UsbPortDetectionArray where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/msg/UsbPortDetectionArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray() }
  }
}


#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__UsbPortObservation() -> *const std::ffi::c_void;
}

#[link(name = "pinkk_usb_insertion_interfaces__rosidl_generator_c")]
extern "C" {
    fn pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(msg: *mut UsbPortObservation) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UsbPortObservation>, size: usize) -> bool;
    fn pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UsbPortObservation>);
    fn pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UsbPortObservation>, out_seq: *mut rosidl_runtime_rs::Sequence<UsbPortObservation>) -> bool;
}

// Corresponds to pinkk_usb_insertion_interfaces__msg__UsbPortObservation
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 검출 keypoint와 solvePnP 결과를 원자적으로 전달한다.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UsbPortObservation {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detection_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub keypoints: [super::super::msg::rmw::Keypoint2D; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub object_confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reprojection_error_px: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub depth_m: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rejection_reason: rosidl_runtime_rs::String,

}



impl Default for UsbPortObservation {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(&mut msg as *mut _) {
        panic!("Call to pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UsbPortObservation {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UsbPortObservation {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UsbPortObservation where Self: Sized {
  const TYPE_NAME: &'static str = "pinkk_usb_insertion_interfaces/msg/UsbPortObservation";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pinkk_usb_insertion_interfaces__msg__UsbPortObservation() }
  }
}


