// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__rosidl_typesupport_introspection_c.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detections`
#include "pinkk_usb_insertion_interfaces/msg/usb_port_detection.h"
// Member `detections`
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(message_memory);
}

void pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(message_memory);
}

size_t pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__size_function__UsbPortDetectionArray__detections(
  const void * untyped_member)
{
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * member =
    (const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence *)(untyped_member);
  return member->size;
}

const void * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__get_const_function__UsbPortDetectionArray__detections(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * member =
    (const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__get_function__UsbPortDetectionArray__detections(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * member =
    (pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__fetch_function__UsbPortDetectionArray__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * item =
    ((const pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__get_const_function__UsbPortDetectionArray__detections(untyped_member, index));
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * value =
    (pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__assign_function__UsbPortDetectionArray__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * item =
    ((pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__get_function__UsbPortDetectionArray__detections(untyped_member, index));
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * value =
    (const pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)(untyped_value);
  *item = *value;
}

bool pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__resize_function__UsbPortDetectionArray__detections(
  void * untyped_member, size_t size)
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * member =
    (pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence *)(untyped_member);
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__fini(member);
  return pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray, detections),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__size_function__UsbPortDetectionArray__detections,  // size() function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__get_const_function__UsbPortDetectionArray__detections,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__get_function__UsbPortDetectionArray__detections,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__fetch_function__UsbPortDetectionArray__detections,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__assign_function__UsbPortDetectionArray__detections,  // assign(index, value) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__resize_function__UsbPortDetectionArray__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_members = {
  "pinkk_usb_insertion_interfaces__msg",  // message namespace
  "UsbPortDetectionArray",  // message name
  2,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, UsbPortDetectionArray)() {
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, UsbPortDetection)();
  if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__rosidl_typesupport_introspection_c__UsbPortDetectionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
