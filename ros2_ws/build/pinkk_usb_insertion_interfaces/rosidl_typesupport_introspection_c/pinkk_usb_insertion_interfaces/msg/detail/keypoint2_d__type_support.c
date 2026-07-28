// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pinkk_usb_insertion_interfaces:msg/Keypoint2D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__rosidl_typesupport_introspection_c.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__msg__Keypoint2D__init(message_memory);
}

void pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__msg__Keypoint2D__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_member_array[5] = {
  {
    "index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__Keypoint2D, index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__Keypoint2D, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__Keypoint2D, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__Keypoint2D, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "visible",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__Keypoint2D, visible),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_members = {
  "pinkk_usb_insertion_interfaces__msg",  // message namespace
  "Keypoint2D",  // message name
  5,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__msg__Keypoint2D),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, Keypoint2D)() {
  if (!pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__msg__Keypoint2D__rosidl_typesupport_introspection_c__Keypoint2D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
