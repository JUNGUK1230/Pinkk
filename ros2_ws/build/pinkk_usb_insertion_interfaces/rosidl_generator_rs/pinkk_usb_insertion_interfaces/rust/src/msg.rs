#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to pinkk_usb_insertion_interfaces__msg__Keypoint2D
/// YOLO가 검출한 하나의 2D keypoint.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Keypoint2D::default())
  }
}

impl rosidl_runtime_rs::Message for Keypoint2D {
  type RmwMsg = super::msg::rmw::Keypoint2D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        index: msg.index,
        x: msg.x,
        y: msg.y,
        confidence: msg.confidence,
        visible: msg.visible,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      index: msg.index,
      x: msg.x,
      y: msg.y,
      confidence: msg.confidence,
      visible: msg.visible,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      index: msg.index,
      x: msg.x,
      y: msg.y,
      confidence: msg.confidence,
      visible: msg.visible,
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__msg__UsbPortDetection
/// 한 USB-A 포트 후보에 대한 YOLO keypoint 결과.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UsbPortDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detection_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub class_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub object_confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bbox: vision_msgs::msg::BoundingBox2D,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_image_width: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_image_height: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub keypoints: [super::msg::Keypoint2D; 4],

}



impl Default for UsbPortDetection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UsbPortDetection::default())
  }
}

impl rosidl_runtime_rs::Message for UsbPortDetection {
  type RmwMsg = super::msg::rmw::UsbPortDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detection_id: msg.detection_id.as_str().into(),
        class_name: msg.class_name.as_str().into(),
        object_confidence: msg.object_confidence,
        bbox: vision_msgs::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Owned(msg.bbox)).into_owned(),
        source_image_width: msg.source_image_width,
        source_image_height: msg.source_image_height,
        keypoints: msg.keypoints
          .map(|elem| super::msg::Keypoint2D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned()),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detection_id: msg.detection_id.as_str().into(),
        class_name: msg.class_name.as_str().into(),
      object_confidence: msg.object_confidence,
        bbox: vision_msgs::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bbox)).into_owned(),
      source_image_width: msg.source_image_width,
      source_image_height: msg.source_image_height,
        keypoints: msg.keypoints
          .iter()
          .map(|elem| super::msg::Keypoint2D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detection_id: msg.detection_id.to_string(),
      class_name: msg.class_name.to_string(),
      object_confidence: msg.object_confidence,
      bbox: vision_msgs::msg::BoundingBox2D::from_rmw_message(msg.bbox),
      source_image_width: msg.source_image_width,
      source_image_height: msg.source_image_height,
      keypoints: msg.keypoints
        .map(super::msg::Keypoint2D::from_rmw_message),
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray
/// 같은 영상 프레임에서 검출된 USB-A 포트 후보 목록.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UsbPortDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: Vec<super::msg::UsbPortDetection>,

}



impl Default for UsbPortDetectionArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UsbPortDetectionArray::default())
  }
}

impl rosidl_runtime_rs::Message for UsbPortDetectionArray {
  type RmwMsg = super::msg::rmw::UsbPortDetectionArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detections: msg.detections
          .into_iter()
          .map(|elem| super::msg::UsbPortDetection::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detections: msg.detections
          .iter()
          .map(|elem| super::msg::UsbPortDetection::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detections: msg.detections
          .into_iter()
          .map(super::msg::UsbPortDetection::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__msg__UsbPortObservation
/// 검출 keypoint와 solvePnP 결과를 원자적으로 전달한다.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UsbPortObservation {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detection_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub keypoints: [super::msg::Keypoint2D; 4],


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
    pub rejection_reason: std::string::String,

}



impl Default for UsbPortObservation {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UsbPortObservation::default())
  }
}

impl rosidl_runtime_rs::Message for UsbPortObservation {
  type RmwMsg = super::msg::rmw::UsbPortObservation;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detection_id: msg.detection_id.as_str().into(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        keypoints: msg.keypoints
          .map(|elem| super::msg::Keypoint2D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned()),
        object_confidence: msg.object_confidence,
        reprojection_error_px: msg.reprojection_error_px,
        depth_m: msg.depth_m,
        valid: msg.valid,
        rejection_reason: msg.rejection_reason.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detection_id: msg.detection_id.as_str().into(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        keypoints: msg.keypoints
          .iter()
          .map(|elem| super::msg::Keypoint2D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
      object_confidence: msg.object_confidence,
      reprojection_error_px: msg.reprojection_error_px,
      depth_m: msg.depth_m,
      valid: msg.valid,
        rejection_reason: msg.rejection_reason.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detection_id: msg.detection_id.to_string(),
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      keypoints: msg.keypoints
        .map(super::msg::Keypoint2D::from_rmw_message),
      object_confidence: msg.object_confidence,
      reprojection_error_px: msg.reprojection_error_px,
      depth_m: msg.depth_m,
      valid: msg.valid,
      rejection_reason: msg.rejection_reason.to_string(),
    }
  }
}


