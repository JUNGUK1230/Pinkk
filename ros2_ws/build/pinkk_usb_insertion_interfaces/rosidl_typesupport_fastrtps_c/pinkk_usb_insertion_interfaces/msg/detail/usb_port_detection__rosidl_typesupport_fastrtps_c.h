// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice
#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
bool cdr_serialize_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
bool cdr_deserialize_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  eprosima::fastcdr::Cdr &,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t get_serialized_size_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t max_serialized_size_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
bool cdr_serialize_key_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t get_serialized_size_key_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t max_serialized_size_key_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pinkk_usb_insertion_interfaces, msg, UsbPortDetection)();

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
