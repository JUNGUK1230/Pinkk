// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__struct.hpp"
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

void UsbPortObservation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pinkk_usb_insertion_interfaces::msg::UsbPortObservation(_init);
}

void UsbPortObservation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pinkk_usb_insertion_interfaces::msg::UsbPortObservation *>(message_memory);
  typed_message->~UsbPortObservation();
}

size_t size_function__UsbPortObservation__keypoints(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__UsbPortObservation__keypoints(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__UsbPortObservation__keypoints(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<pinkk_usb_insertion_interfaces::msg::Keypoint2D, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__UsbPortObservation__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(
    get_const_function__UsbPortObservation__keypoints(untyped_member, index));
  auto & value = *reinterpret_cast<pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(untyped_value);
  value = item;
}

void assign_function__UsbPortObservation__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(
    get_function__UsbPortObservation__keypoints(untyped_member, index));
  const auto & value = *reinterpret_cast<const pinkk_usb_insertion_interfaces::msg::Keypoint2D *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember UsbPortObservation_message_member_array[9] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, header),  // bytes offset in struct
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
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, detection_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, pose),  // bytes offset in struct
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
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, keypoints),  // bytes offset in struct
    nullptr,  // default value
    size_function__UsbPortObservation__keypoints,  // size() function pointer
    get_const_function__UsbPortObservation__keypoints,  // get_const(index) function pointer
    get_function__UsbPortObservation__keypoints,  // get(index) function pointer
    fetch_function__UsbPortObservation__keypoints,  // fetch(index, &value) function pointer
    assign_function__UsbPortObservation__keypoints,  // assign(index, value) function pointer
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
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, object_confidence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reprojection_error_px",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, reprojection_error_px),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "depth_m",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, depth_m),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rejection_reason",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation, rejection_reason),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers UsbPortObservation_message_members = {
  "pinkk_usb_insertion_interfaces::msg",  // message namespace
  "UsbPortObservation",  // message name
  9,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces::msg::UsbPortObservation),
  false,  // has_any_key_member_
  UsbPortObservation_message_member_array,  // message members
  UsbPortObservation_init_function,  // function to initialize message memory (memory has to be allocated)
  UsbPortObservation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t UsbPortObservation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &UsbPortObservation_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::msg::UsbPortObservation>()
{
  return &::pinkk_usb_insertion_interfaces::msg::rosidl_typesupport_introspection_cpp::UsbPortObservation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, msg, UsbPortObservation)() {
  return &::pinkk_usb_insertion_interfaces::msg::rosidl_typesupport_introspection_cpp::UsbPortObservation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
