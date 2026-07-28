// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_observation.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__STRUCT_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'keypoints'
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortObservation __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortObservation __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UsbPortObservation_
{
  using Type = UsbPortObservation_<ContainerAllocator>;

  explicit UsbPortObservation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_id = "";
      this->keypoints.fill(pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>{_init});
      this->object_confidence = 0.0f;
      this->reprojection_error_px = 0.0f;
      this->depth_m = 0.0f;
      this->valid = false;
      this->rejection_reason = "";
    }
  }

  explicit UsbPortObservation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    detection_id(_alloc),
    pose(_alloc, _init),
    keypoints(_alloc),
    rejection_reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_id = "";
      this->keypoints.fill(pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>{_alloc, _init});
      this->object_confidence = 0.0f;
      this->reprojection_error_px = 0.0f;
      this->depth_m = 0.0f;
      this->valid = false;
      this->rejection_reason = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detection_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _detection_id_type detection_id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _keypoints_type =
    std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>, 4>;
  _keypoints_type keypoints;
  using _object_confidence_type =
    float;
  _object_confidence_type object_confidence;
  using _reprojection_error_px_type =
    float;
  _reprojection_error_px_type reprojection_error_px;
  using _depth_m_type =
    float;
  _depth_m_type depth_m;
  using _valid_type =
    bool;
  _valid_type valid;
  using _rejection_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _rejection_reason_type rejection_reason;

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
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__keypoints(
    const std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>, 4> & _arg)
  {
    this->keypoints = _arg;
    return *this;
  }
  Type & set__object_confidence(
    const float & _arg)
  {
    this->object_confidence = _arg;
    return *this;
  }
  Type & set__reprojection_error_px(
    const float & _arg)
  {
    this->reprojection_error_px = _arg;
    return *this;
  }
  Type & set__depth_m(
    const float & _arg)
  {
    this->depth_m = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__rejection_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->rejection_reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortObservation
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortObservation
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UsbPortObservation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detection_id != other.detection_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->keypoints != other.keypoints) {
      return false;
    }
    if (this->object_confidence != other.object_confidence) {
      return false;
    }
    if (this->reprojection_error_px != other.reprojection_error_px) {
      return false;
    }
    if (this->depth_m != other.depth_m) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->rejection_reason != other.rejection_reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const UsbPortObservation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UsbPortObservation_

// alias to use template instance with default allocator
using UsbPortObservation =
  pinkk_usb_insertion_interfaces::msg::UsbPortObservation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__STRUCT_HPP_
