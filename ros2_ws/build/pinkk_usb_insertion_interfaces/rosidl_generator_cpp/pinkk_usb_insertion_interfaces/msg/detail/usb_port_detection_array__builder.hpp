// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection_array.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__BUILDER_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

namespace builder
{

class Init_UsbPortDetectionArray_detections
{
public:
  explicit Init_UsbPortDetectionArray_detections(::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray detections(::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray msg_;
};

class Init_UsbPortDetectionArray_header
{
public:
  Init_UsbPortDetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UsbPortDetectionArray_detections header(::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UsbPortDetectionArray_detections(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray>()
{
  return pinkk_usb_insertion_interfaces::msg::builder::Init_UsbPortDetectionArray_header();
}

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__BUILDER_HPP_
