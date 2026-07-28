// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection_array.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__STRUCT_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__STRUCT_HPP_

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
// Member 'detections'
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UsbPortDetectionArray_
{
  using Type = UsbPortDetectionArray_<ContainerAllocator>;

  explicit UsbPortDetectionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit UsbPortDetectionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detections_type =
    std::vector<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::msg::UsbPortDetection_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UsbPortDetectionArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const UsbPortDetectionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UsbPortDetectionArray_

// alias to use template instance with default allocator
using UsbPortDetectionArray =
  pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__STRUCT_HPP_
