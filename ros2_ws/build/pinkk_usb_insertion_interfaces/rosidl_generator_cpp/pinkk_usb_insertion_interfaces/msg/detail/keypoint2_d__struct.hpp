// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinkk_usb_insertion_interfaces:msg/Keypoint2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/keypoint2_d.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__STRUCT_HPP_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__Keypoint2D __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__msg__Keypoint2D __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Keypoint2D_
{
  using Type = Keypoint2D_<ContainerAllocator>;

  explicit Keypoint2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->x = 0.0;
      this->y = 0.0;
      this->confidence = 0.0f;
      this->visible = false;
    }
  }

  explicit Keypoint2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->x = 0.0;
      this->y = 0.0;
      this->confidence = 0.0f;
      this->visible = false;
    }
  }

  // field types and members
  using _index_type =
    uint8_t;
  _index_type index;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _visible_type =
    bool;
  _visible_type visible;

  // setters for named parameter idiom
  Type & set__index(
    const uint8_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__visible(
    const bool & _arg)
  {
    this->visible = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__Keypoint2D
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__msg__Keypoint2D
    std::shared_ptr<pinkk_usb_insertion_interfaces::msg::Keypoint2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Keypoint2D_ & other) const
  {
    if (this->index != other.index) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->visible != other.visible) {
      return false;
    }
    return true;
  }
  bool operator!=(const Keypoint2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Keypoint2D_

// alias to use template instance with default allocator
using Keypoint2D =
  pinkk_usb_insertion_interfaces::msg::Keypoint2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__STRUCT_HPP_
