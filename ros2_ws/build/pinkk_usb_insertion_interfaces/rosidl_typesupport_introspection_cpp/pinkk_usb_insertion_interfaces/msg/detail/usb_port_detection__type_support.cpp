// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pinkk_usb_insertion_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void UsbPortDetection_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pinkk_usb_insertion_interfaces::msg::UsbPortDetection(_init);
}

void UsbPortDetection_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pinkk_usb_insertion_interfaces::msg::UsbPortDetection *>(message_memory);
  typed_message->~UsbPortDetection();
}

size_t size_function__UsbPortDetection__keypoints(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__UsbPortDetection__keypoints(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__UsbPortDetection__keypoints(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__UsbPortDetection__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(
    get_const_function__UsbPortDetection__keypoints(untyped_member, index));
  auto & value = *reinterpret_cast<pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(untyped_value);
  value = item;
}

void assign_function__UsbPortDetection__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(
    get_function__UsbPortDetection__keypoints(untyped_member, index));
  const auto & value = *reinterpret_cast<const pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember UsbPortDetection_message_member_array[8] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "detection_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, detection_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "class_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, class_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "object_confidence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, object_confidence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bbox",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<vision_msgs::msg::BoundingBox2D>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, bbox),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "source_image_width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, source_image_width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "source_image_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, source_image_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::msg::Keypoint2D>(),  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection, keypoints),  // bytes offset in struct
    nullptr,  // default value
    size_function__UsbPortDetection__keypoints,  // size() function pointer
    get_const_function__UsbPortDetection__keypoints,  // get_const(index) function pointer
    get_function__UsbPortDetection__keypoints,  // get(index) function pointer
    fetch_function__UsbPortDetection__keypoints,  // fetch(index, &value) function pointer
    assign_function__UsbPortDetection__keypoints,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers UsbPortDetection_message_members = {
  "pinkk_usb_insertion_interfaces::msg",  // message namespace
  "UsbPortDetection",  // message name
  8,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces::msg::UsbPortDetection),
  false,  // has_any_key_member_
  UsbPortDetection_message_member_array,  // message members
  UsbPortDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  UsbPortDetection_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t UsbPortDetection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &UsbPortDetection_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>()
{
  return &::pinkk_usb_insertion_interfaces::msg::rosidl_typesupport_introspection_cpp::UsbPortDetection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, msg, UsbPortDetection)() {
  return &::pinkk_usb_insertion_interfaces::msg::rosidl_typesupport_introspection_cpp::UsbPortDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
