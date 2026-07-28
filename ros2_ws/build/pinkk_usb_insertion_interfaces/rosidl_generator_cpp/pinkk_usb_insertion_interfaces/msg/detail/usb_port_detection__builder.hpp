// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__BUILDER_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

namespace builder
{

class Init_UsbPortDetection_keypoints
{
public:
  explicit Init_UsbPortDetection_keypoints(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection keypoints(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_source_image_height
{
public:
  explicit Init_UsbPortDetection_source_image_height(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  Init_UsbPortDetection_keypoints source_image_height(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_source_image_height_type arg)
  {
    msg_.source_image_height = std::move(arg);
    return Init_UsbPortDetection_keypoints(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_source_image_width
{
public:
  explicit Init_UsbPortDetection_source_image_width(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  Init_UsbPortDetection_source_image_height source_image_width(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_source_image_width_type arg)
  {
    msg_.source_image_width = std::move(arg);
    return Init_UsbPortDetection_source_image_height(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_bbox
{
public:
  explicit Init_UsbPortDetection_bbox(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  Init_UsbPortDetection_source_image_width bbox(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_bbox_type arg)
  {
    msg_.bbox = std::move(arg);
    return Init_UsbPortDetection_source_image_width(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_object_confidence
{
public:
  explicit Init_UsbPortDetection_object_confidence(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  Init_UsbPortDetection_bbox object_confidence(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_object_confidence_type arg)
  {
    msg_.object_confidence = std::move(arg);
    return Init_UsbPortDetection_bbox(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_class_name
{
public:
  explicit Init_UsbPortDetection_class_name(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  Init_UsbPortDetection_object_confidence class_name(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_UsbPortDetection_object_confidence(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_detection_id
{
public:
  explicit Init_UsbPortDetection_detection_id(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
  : msg_(msg)
  {}
  Init_UsbPortDetection_class_name detection_id(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_detection_id_type arg)
  {
    msg_.detection_id = std::move(arg);
    return Init_UsbPortDetection_class_name(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

class Init_UsbPortDetection_header
{
public:
  Init_UsbPortDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UsbPortDetection_detection_id header(::pinkk_usb_insertion_interfaces::msg::UsbPortDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UsbPortDetection_detection_id(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::msg::UsbPortDetection>()
{
  return pinkk_usb_insertion_interfaces::msg::builder::Init_UsbPortDetection_header();
}

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__BUILDER_HPP_
