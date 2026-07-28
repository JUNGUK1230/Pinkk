// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/Keypoint2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/keypoint2_d.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__TRAITS_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Keypoint2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: visible
  {
    out << "visible: ";
    rosidl_generator_traits::value_to_yaml(msg.visible, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Keypoint2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visible: ";
    rosidl_generator_traits::value_to_yaml(msg.visible, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Keypoint2D & msg, bool use_flow_style = false)
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

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pinkk_usb_insertion_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pinkk_usb_insertion_interfaces::msg::Keypoint2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinkk_usb_insertion_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinkk_usb_insertion_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pinkk_usb_insertion_interfaces::msg::Keypoint2D & msg)
{
  return pinkk_usb_insertion_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pinkk_usb_insertion_interfaces::msg::Keypoint2D>()
{
  return "pinkk_usb_insertion_interfaces::msg::Keypoint2D";
}

template<>
inline const char * name<pinkk_usb_insertion_interfaces::msg::Keypoint2D>()
{
  return "pinkk_usb_insertion_interfaces/msg/Keypoint2D";
}

template<>
struct has_fixed_size<pinkk_usb_insertion_interfaces::msg::Keypoint2D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pinkk_usb_insertion_interfaces::msg::Keypoint2D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pinkk_usb_insertion_interfaces::msg::Keypoint2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__TRAITS_HPP_
