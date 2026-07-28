// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_observation.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__BUILDER_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

namespace builder
{

class Init_UsbPortObservation_rejection_reason
{
public:
  explicit Init_UsbPortObservation_rejection_reason(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation rejection_reason(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_rejection_reason_type arg)
  {
    msg_.rejection_reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_valid
{
public:
  explicit Init_UsbPortObservation_valid(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_rejection_reason valid(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_UsbPortObservation_rejection_reason(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_depth_m
{
public:
  explicit Init_UsbPortObservation_depth_m(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_valid depth_m(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_depth_m_type arg)
  {
    msg_.depth_m = std::move(arg);
    return Init_UsbPortObservation_valid(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_reprojection_error_px
{
public:
  explicit Init_UsbPortObservation_reprojection_error_px(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_depth_m reprojection_error_px(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_reprojection_error_px_type arg)
  {
    msg_.reprojection_error_px = std::move(arg);
    return Init_UsbPortObservation_depth_m(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_object_confidence
{
public:
  explicit Init_UsbPortObservation_object_confidence(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_reprojection_error_px object_confidence(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_object_confidence_type arg)
  {
    msg_.object_confidence = std::move(arg);
    return Init_UsbPortObservation_reprojection_error_px(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_keypoints
{
public:
  explicit Init_UsbPortObservation_keypoints(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_object_confidence keypoints(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return Init_UsbPortObservation_object_confidence(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_pose
{
public:
  explicit Init_UsbPortObservation_pose(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_keypoints pose(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_UsbPortObservation_keypoints(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_detection_id
{
public:
  explicit Init_UsbPortObservation_detection_id(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
  : msg_(msg)
  {}
  Init_UsbPortObservation_pose detection_id(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_detection_id_type arg)
  {
    msg_.detection_id = std::move(arg);
    return Init_UsbPortObservation_pose(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

class Init_UsbPortObservation_header
{
public:
  Init_UsbPortObservation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UsbPortObservation_detection_id header(::pinkk_usb_insertion_interfaces::msg::UsbPortObservation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UsbPortObservation_detection_id(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortObservation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::msg::UsbPortObservation>()
{
  return pinkk_usb_insertion_interfaces::msg::builder::Init_UsbPortObservation_header();
}

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__BUILDER_HPP_
