// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__rosidl_typesupport_introspection_c.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detection_id`
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `bbox`
#include "vision_msgs/msg/bounding_box2_d.h"
// Member `bbox`
#include "vision_msgs/msg/detail/bounding_box2_d__rosidl_typesupport_introspection_c.h"
// Member `keypoints`
#include "pinkk_usb_insertion_interfaces/msg/keypoint2_d.h"
// Member `keypoints`
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(message_memory);
}

void pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(message_memory);
}

size_t pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__size_function__UsbPortDetection__keypoints(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__get_const_function__UsbPortDetection__keypoints(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * member =
    (const pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_member);
  return &member[index];
}

void * pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__get_function__UsbPortDetection__keypoints(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * member =
    (pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_member);
  return &member[index];
}

void pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__fetch_function__UsbPortDetection__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * item =
    ((const pinkk_usb_insertion_interfaces__msg__Keypoint2D *)
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__get_const_function__UsbPortDetection__keypoints(untyped_member, index));
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * value =
    (pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__assign_function__UsbPortDetection__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * item =
    ((pinkk_usb_insertion_interfaces__msg__Keypoint2D *)
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__get_function__UsbPortDetection__keypoints(untyped_member, index));
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * value =
    (const pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detection_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, detection_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "class_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, class_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, object_confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, bbox),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_image_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, source_image_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_image_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, source_image_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection, keypoints),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__size_function__UsbPortDetection__keypoints,  // size() function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__get_const_function__UsbPortDetection__keypoints,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__get_function__UsbPortDetection__keypoints,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__fetch_function__UsbPortDetection__keypoints,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__assign_function__UsbPortDetection__keypoints,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_members = {
  "pinkk_usb_insertion_interfaces__msg",  // message namespace
  "UsbPortDetection",  // message name
  8,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, UsbPortDetection)() {
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, BoundingBox2D)();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, Keypoint2D)();
  if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__rosidl_typesupport_introspection_c__UsbPortDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
