// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_observation.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__TRAITS_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'keypoints'
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__traits.hpp"

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const UsbPortObservation & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: detection_id
  {
    out << "detection_id: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_id, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: keypoints
  {
    if (msg.keypoints.size() == 0) {
      out << "keypoints: []";
    } else {
      out << "keypoints: [";
      size_t pending_items = msg.keypoints.size();
      for (auto item : msg.keypoints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: object_confidence
  {
    out << "object_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.object_confidence, out);
    out << ", ";
  }

  // member: reprojection_error_px
  {
    out << "reprojection_error_px: ";
    rosidl_generator_traits::value_to_yaml(msg.reprojection_error_px, out);
    out << ", ";
  }

  // member: depth_m
  {
    out << "depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_m, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: rejection_reason
  {
    out << "rejection_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.rejection_reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UsbPortObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: detection_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_id: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: keypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.keypoints.size() == 0) {
      out << "keypoints: []\n";
    } else {
      out << "keypoints:\n";
      for (auto item : msg.keypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: object_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.object_confidence, out);
    out << "\n";
  }

  // member: reprojection_error_px
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reprojection_error_px: ";
    rosidl_generator_traits::value_to_yaml(msg.reprojection_error_px, out);
    out << "\n";
  }

  // member: depth_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_m, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: rejection_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rejection_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.rejection_reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UsbPortObservation & msg, bool use_flow_style = false)
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
  const pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinkk_usb_insertion_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinkk_usb_insertion_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pinkk_usb_insertion_interfaces::msg::UsbPortObservation & msg)
{
  return pinkk_usb_insertion_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pinkk_usb_insertion_interfaces::msg::UsbPortObservation>()
{
  return "pinkk_usb_insertion_interfaces::msg::UsbPortObservation";
}

template<>
inline const char * name<pinkk_usb_insertion_interfaces::msg::UsbPortObservation>()
{
  return "pinkk_usb_insertion_interfaces/msg/UsbPortObservation";
}

template<>
struct has_fixed_size<pinkk_usb_insertion_interfaces::msg::UsbPortObservation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pinkk_usb_insertion_interfaces::msg::UsbPortObservation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pinkk_usb_insertion_interfaces::msg::UsbPortObservation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__TRAITS_HPP_
