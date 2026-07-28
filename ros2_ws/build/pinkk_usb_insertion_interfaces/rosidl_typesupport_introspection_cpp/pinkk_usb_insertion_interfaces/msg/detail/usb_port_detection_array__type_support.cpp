// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__struct.hpp"
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

void UsbPortDetectionArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray(_init);
}

void UsbPortDetectionArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray *>(message_memory);
  typed_message->~UsbPortDetectionArray();
}

size_t size_function__UsbPortDetectionArray__detections(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<pinkk_usb_insertion_interfaces::msg::UsbPortDetection> *>(untyped_member);
  return member->size();
}

const void * get_const_function__UsbPortDetectionArray__detections(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<pinkk_usb_insertion_interfaces::msg::UsbPortDetection> *>(untyped_member);
  return &member[index];
}

void * get_function__UsbPortDetectionArray__detections(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<pinkk_usb_insertion_interfaces::msg::UsbPortDetection> *>(untyped_member);
  return &member[index];
}

void fetch_function__UsbPortDetectionArray__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const pinkk_usb_insertion_interfaces::msg::UsbPortDetection *>(
    get_const_function__UsbPortDetectionArray__detections(untyped_member, index));
  auto & value = *reinterpret_cast<pinkk_usb_insertion_interfaces::msg::UsbPortDetection *>(untyped_value);
  value = item;
}

void assign_function__UsbPortDetectionArray__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<pinkk_usb_insertion_interfaces::msg::UsbPortDetection *>(
    get_function__UsbPortDetectionArray__detections(untyped_member, index));
  const auto & value = *reinterpret_cast<const pinkk_usb_insertion_interfaces::msg::UsbPortDetection *>(untyped_value);
  item = value;
}

void resize_function__UsbPortDetectionArray__detections(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<pinkk_usb_insertion_interfaces::msg::UsbPortDetection> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember UsbPortDetectionArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "detections",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::msg::UsbPortDetection>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray, detections),  // bytes offset in struct
    nullptr,  // default value
    size_function__UsbPortDetectionArray__detections,  // size() function pointer
    get_const_function__UsbPortDetectionArray__detections,  // get_const(index) function pointer
    get_function__UsbPortDetectionArray__detections,  // get(index) function pointer
    fetch_function__UsbPortDetectionArray__detections,  // fetch(index, &value) function pointer
    assign_function__UsbPortDetectionArray__detections,  // assign(index, value) function pointer
    resize_function__UsbPortDetectionArray__detections  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers UsbPortDetectionArray_message_members = {
  "pinkk_usb_insertion_interfaces::msg",  // message namespace
  "UsbPortDetectionArray",  // message name
  2,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray),
  false,  // has_any_key_member_
  UsbPortDetectionArray_message_member_array,  // message members
  UsbPortDetectionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  UsbPortDetectionArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t UsbPortDetectionArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &UsbPortDetectionArray_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace pinkk_usb_insertion_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::msg::UsbPortDetectionArray>()
{
  return &::pinkk_usb_insertion_interfaces::msg::rosidl_typesupport_introspection_cpp::UsbPortDetectionArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, msg, UsbPortDetectionArray)() {
  return &::pinkk_usb_insertion_interfaces::msg::rosidl_typesupport_introspection_cpp::UsbPortDetectionArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
