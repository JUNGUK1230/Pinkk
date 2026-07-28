// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__TRAITS_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'bbox'
#include "vision_msgs/msg/detail/bounding_box2_d__traits.hpp"
// Member 'keypoints'
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__traits.hpp"

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const UsbPortDetection & msg,
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

  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: object_confidence
  {
    out << "object_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.object_confidence, out);
    out << ", ";
  }

  // member: bbox
  {
    out << "bbox: ";
    to_flow_style_yaml(msg.bbox, out);
    out << ", ";
  }

  // member: source_image_width
  {
    out << "source_image_width: ";
    rosidl_generator_traits::value_to_yaml(msg.source_image_width, out);
    out << ", ";
  }

  // member: source_image_height
  {
    out << "source_image_height: ";
    rosidl_generator_traits::value_to_yaml(msg.source_image_height, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UsbPortDetection & msg,
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

  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
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

  // member: bbox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox:\n";
    to_block_style_yaml(msg.bbox, out, indentation + 2);
  }

  // member: source_image_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_image_width: ";
    rosidl_generator_traits::value_to_yaml(msg.source_image_width, out);
    out << "\n";
  }

  // member: source_image_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_image_height: ";
    rosidl_generator_traits::value_to_yaml(msg.source_image_height, out);
    out << "\n";
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UsbPortDetection & msg, bool use_flow_style = false)
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
  const pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinkk_usb_insertion_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinkk_usb_insertion_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pinkk_usb_insertion_interfaces::msg::UsbPortDetection & msg)
{
  return pinkk_usb_insertion_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>()
{
  return "pinkk_usb_insertion_interfaces::msg::UsbPortDetection";
}

template<>
inline const char * name<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>()
{
  return "pinkk_usb_insertion_interfaces/msg/UsbPortDetection";
}

template<>
struct has_fixed_size<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__TRAITS_HPP_
