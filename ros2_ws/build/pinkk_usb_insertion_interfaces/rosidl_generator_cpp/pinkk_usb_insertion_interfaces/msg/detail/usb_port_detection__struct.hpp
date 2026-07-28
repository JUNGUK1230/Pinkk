// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__STRUCT_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'bbox'
#include "vision_msgs/msg/detail/bounding_box2_d__struct.hpp"
// Member 'keypoints'
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetection __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetection __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UsbPortDetection_
{
  using Type = UsbPortDetection_<ContainerAllocator>;

  explicit UsbPortDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    bbox(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_id = "";
      this->class_name = "";
      this->object_confidence = 0.0f;
      this->source_image_width = 0ul;
      this->source_image_height = 0ul;
      this->keypoints.fill(pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>{_init});
    }
  }

  explicit UsbPortDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    detection_id(_alloc),
    class_name(_alloc),
    bbox(_alloc, _init),
    keypoints(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_id = "";
      this->class_name = "";
      this->object_confidence = 0.0f;
      this->source_image_width = 0ul;
      this->source_image_height = 0ul;
      this->keypoints.fill(pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detection_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _detection_id_type detection_id;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _object_confidence_type =
    float;
  _object_confidence_type object_confidence;
  using _bbox_type =
    vision_msgs::msg::BoundingBox2D_<ContainerAllocator>;
  _bbox_type bbox;
  using _source_image_width_type =
    uint32_t;
  _source_image_width_type source_image_width;
  using _source_image_height_type =
    uint32_t;
  _source_image_height_type source_image_height;
  using _keypoints_type =
    std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>, 4>;
  _keypoints_type keypoints;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detection_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->detection_id = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__object_confidence(
    const float & _arg)
  {
    this->object_confidence = _arg;
    return *this;
  }
  Type & set__bbox(
    const vision_msgs::msg::BoundingBox2D_<ContainerAllocator> & _arg)
  {
    this->bbox = _arg;
    return *this;
  }
  Type & set__source_image_width(
    const uint32_t & _arg)
  {
    this->source_image_width = _arg;
    return *this;
  }
  Type & set__source_image_height(
    const uint32_t & _arg)
  {
    this->source_image_height = _arg;
    return *this;
  }
  Type & set__keypoints(
    const std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>, 4> & _arg)
  {
    this->keypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetection
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetection
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UsbPortDetection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detection_id != other.detection_id) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->object_confidence != other.object_confidence) {
      return false;
    }
    if (this->bbox != other.bbox) {
      return false;
    }
    if (this->source_image_width != other.source_image_width) {
      return false;
    }
    if (this->source_image_height != other.source_image_height) {
      return false;
    }
    if (this->keypoints != other.keypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const UsbPortDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UsbPortDetection_

// alias to use template instance with default allocator
using UsbPortDetection =
  pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__STRUCT_HPP_
