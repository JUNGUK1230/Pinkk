// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/Keypoint2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/keypoint2_d.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__BUILDER_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

namespace builder
{

class Init_Keypoint2D_visible
{
public:
  explicit Init_Keypoint2D_visible(::pinkk_usb_insertion_interfaces::msg::Keypoint2D & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::msg::Keypoint2D visible(::pinkk_usb_insertion_interfaces::msg::Keypoint2D::_visible_type arg)
  {
    msg_.visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::Keypoint2D msg_;
};

class Init_Keypoint2D_confidence
{
public:
  explicit Init_Keypoint2D_confidence(::pinkk_usb_insertion_interfaces::msg::Keypoint2D & msg)
  : msg_(msg)
  {}
  Init_Keypoint2D_visible confidence(::pinkk_usb_insertion_interfaces::msg::Keypoint2D::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Keypoint2D_visible(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::Keypoint2D msg_;
};

class Init_Keypoint2D_y
{
public:
  explicit Init_Keypoint2D_y(::pinkk_usb_insertion_interfaces::msg::Keypoint2D & msg)
  : msg_(msg)
  {}
  Init_Keypoint2D_confidence y(::pinkk_usb_insertion_interfaces::msg::Keypoint2D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Keypoint2D_confidence(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::Keypoint2D msg_;
};

class Init_Keypoint2D_x
{
public:
  explicit Init_Keypoint2D_x(::pinkk_usb_insertion_interfaces::msg::Keypoint2D & msg)
  : msg_(msg)
  {}
  Init_Keypoint2D_y x(::pinkk_usb_insertion_interfaces::msg::Keypoint2D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Keypoint2D_y(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::Keypoint2D msg_;
};

class Init_Keypoint2D_index
{
public:
  Init_Keypoint2D_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Keypoint2D_x index(::pinkk_usb_insertion_interfaces::msg::Keypoint2D::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_Keypoint2D_x(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::msg::Keypoint2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::msg::Keypoint2D>()
{
  return pinkk_usb_insertion_interfaces::msg::builder::Init_Keypoint2D_index();
}

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__BUILDER_HPP_
