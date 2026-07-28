// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"  // keypoints
#include "rosidl_runtime_c/string.h"  // class_name, detection_id
#include "rosidl_runtime_c/string_functions.h"  // class_name, detection_id
#include "std_msgs/msg/detail/header__functions.h"  // header
#include "vision_msgs/msg/detail/bounding_box2_d__functions.h"  // bbox

// forward declare type support functions

bool cdr_serialize_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  eprosima::fastcdr::Cdr & cdr,
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * ros_message);

size_t get_serialized_size_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pinkk_usb_insertion_interfaces, msg, Keypoint2D)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
bool cdr_serialize_vision_msgs__msg__BoundingBox2D(
  const vision_msgs__msg__BoundingBox2D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
bool cdr_deserialize_vision_msgs__msg__BoundingBox2D(
  eprosima::fastcdr::Cdr & cdr,
  vision_msgs__msg__BoundingBox2D * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t get_serialized_size_vision_msgs__msg__BoundingBox2D(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t max_serialized_size_vision_msgs__msg__BoundingBox2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
bool cdr_serialize_key_vision_msgs__msg__BoundingBox2D(
  const vision_msgs__msg__BoundingBox2D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t get_serialized_size_key_vision_msgs__msg__BoundingBox2D(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
size_t max_serialized_size_key_vision_msgs__msg__BoundingBox2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vision_msgs, msg, BoundingBox2D)();


using _UsbPortDetection__ros_msg_type = pinkk_usb_insertion_interfaces__msg__UsbPortDetection;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
bool cdr_serialize_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: detection_id
  {
    const rosidl_runtime_c__String * str = &ros_message->detection_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: class_name
  {
    const rosidl_runtime_c__String * str = &ros_message->class_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: object_confidence
  {
    cdr << ros_message->object_confidence;
  }

  // Field name: bbox
  {
    cdr_serialize_vision_msgs__msg__BoundingBox2D(
      &ros_message->bbox, cdr);
  }

  // Field name: source_image_width
  {
    cdr << ros_message->source_image_width;
  }

  // Field name: source_image_height
  {
    cdr << ros_message->source_image_height;
  }

  // Field name: keypoints
  {
    size_t size = 4;
    auto array_ptr = ros_message->keypoints;
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
bool cdr_deserialize_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  eprosima::fastcdr::Cdr & cdr,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: detection_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->detection_id.data) {
      rosidl_runtime_c__String__init(&ros_message->detection_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->detection_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'detection_id'\n");
      return false;
    }
  }

  // Field name: class_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->class_name.data) {
      rosidl_runtime_c__String__init(&ros_message->class_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->class_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'class_name'\n");
      return false;
    }
  }

  // Field name: object_confidence
  {
    cdr >> ros_message->object_confidence;
  }

  // Field name: bbox
  {
    cdr_deserialize_vision_msgs__msg__BoundingBox2D(cdr, &ros_message->bbox);
  }

  // Field name: source_image_width
  {
    cdr >> ros_message->source_image_width;
  }

  // Field name: source_image_height
  {
    cdr >> ros_message->source_image_height;
  }

  // Field name: keypoints
  {
    size_t size = 4;
    auto array_ptr = ros_message->keypoints;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_pinkk_usb_insertion_interfaces__msg__Keypoint2D(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t get_serialized_size_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UsbPortDetection__ros_msg_type * ros_message = static_cast<const _UsbPortDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: detection_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->detection_id.size + 1);

  // Field name: class_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->class_name.size + 1);

  // Field name: object_confidence
  {
    size_t item_size = sizeof(ros_message->object_confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: bbox
  current_alignment += get_serialized_size_vision_msgs__msg__BoundingBox2D(
    &(ros_message->bbox), current_alignment);

  // Field name: source_image_width
  {
    size_t item_size = sizeof(ros_message->source_image_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: source_image_height
  {
    size_t item_size = sizeof(ros_message->source_image_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: keypoints
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->keypoints;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t max_serialized_size_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: detection_id
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: class_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: object_confidence
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: bbox
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_vision_msgs__msg__BoundingBox2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: source_image_width
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: source_image_height
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: keypoints
  {
    size_t array_size = 4;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pinkk_usb_insertion_interfaces__msg__UsbPortDetection;
    is_plain =
      (
      offsetof(DataType, keypoints) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
bool cdr_serialize_key_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: detection_id
  {
    const rosidl_runtime_c__String * str = &ros_message->detection_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: class_name
  {
    const rosidl_runtime_c__String * str = &ros_message->class_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: object_confidence
  {
    cdr << ros_message->object_confidence;
  }

  // Field name: bbox
  {
    cdr_serialize_key_vision_msgs__msg__BoundingBox2D(
      &ros_message->bbox, cdr);
  }

  // Field name: source_image_width
  {
    cdr << ros_message->source_image_width;
  }

  // Field name: source_image_height
  {
    cdr << ros_message->source_image_height;
  }

  // Field name: keypoints
  {
    size_t size = 4;
    auto array_ptr = ros_message->keypoints;
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t get_serialized_size_key_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UsbPortDetection__ros_msg_type * ros_message = static_cast<const _UsbPortDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: detection_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->detection_id.size + 1);

  // Field name: class_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->class_name.size + 1);

  // Field name: object_confidence
  {
    size_t item_size = sizeof(ros_message->object_confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: bbox
  current_alignment += get_serialized_size_key_vision_msgs__msg__BoundingBox2D(
    &(ros_message->bbox), current_alignment);

  // Field name: source_image_width
  {
    size_t item_size = sizeof(ros_message->source_image_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: source_image_height
  {
    size_t item_size = sizeof(ros_message->source_image_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: keypoints
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->keypoints;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pinkk_usb_insertion_interfaces
size_t max_serialized_size_key_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: detection_id
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: class_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: object_confidence
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: bbox
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_vision_msgs__msg__BoundingBox2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: source_image_width
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: source_image_height
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: keypoints
  {
    size_t array_size = 4;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_pinkk_usb_insertion_interfaces__msg__Keypoint2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pinkk_usb_insertion_interfaces__msg__UsbPortDetection;
    is_plain =
      (
      offsetof(DataType, keypoints) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _UsbPortDetection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message = static_cast<const pinkk_usb_insertion_interfaces__msg__UsbPortDetection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(ros_message, cdr);
}

static bool _UsbPortDetection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message = static_cast<pinkk_usb_insertion_interfaces__msg__UsbPortDetection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(cdr, ros_message);
}

static uint32_t _UsbPortDetection__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
      untyped_ros_message, 0));
}

static size_t _UsbPortDetection__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pinkk_usb_insertion_interfaces__msg__UsbPortDetection(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UsbPortDetection = {
  "pinkk_usb_insertion_interfaces::msg",
  "UsbPortDetection",
  _UsbPortDetection__cdr_serialize,
  _UsbPortDetection__cdr_deserialize,
  _UsbPortDetection__get_serialized_size,
  _UsbPortDetection__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _UsbPortDetection__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UsbPortDetection,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pinkk_usb_insertion_interfaces, msg, UsbPortDetection)() {
  return &_UsbPortDetection__type_support;
}

#if defined(__cplusplus)
}
#endif
