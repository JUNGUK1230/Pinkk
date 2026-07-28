// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_type_hash_t *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8d, 0x1e, 0x40, 0x68, 0x91, 0x69, 0x11, 0x77,
      0x99, 0xb6, 0x83, 0xbc, 0x75, 0x12, 0x99, 0xa0,
      0x2d, 0x65, 0xbb, 0x25, 0x3f, 0x09, 0x90, 0xb6,
      0xbc, 0x44, 0x95, 0x6c, 0x3b, 0x16, 0xa8, 0xe6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"
#include "vision_msgs/msg/detail/point2_d__functions.h"
#include "vision_msgs/msg/detail/pose2_d__functions.h"
#include "vision_msgs/msg/detail/bounding_box2_d__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t pinkk_usb_insertion_interfaces__msg__Keypoint2D__EXPECTED_HASH = {1, {
    0x64, 0x6b, 0x09, 0x1d, 0x0a, 0xea, 0x80, 0x8a,
    0x3a, 0x5d, 0x00, 0x84, 0xfa, 0xf2, 0x13, 0x7f,
    0x61, 0x3e, 0x49, 0xe0, 0xf9, 0x88, 0xc2, 0xa2,
    0x9e, 0x5f, 0x83, 0x57, 0xe2, 0xb4, 0x34, 0x30,
  }};
static const rosidl_type_hash_t pinkk_usb_insertion_interfaces__msg__UsbPortDetection__EXPECTED_HASH = {1, {
    0xbd, 0x70, 0x33, 0x5b, 0xd0, 0x08, 0x2a, 0xd9,
    0x73, 0x37, 0xca, 0xc1, 0xab, 0xbf, 0x86, 0x03,
    0xdc, 0x11, 0x7a, 0xdb, 0x06, 0x47, 0x63, 0x0c,
    0x57, 0xb0, 0x76, 0xe8, 0x04, 0x6c, 0x52, 0xb3,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
static const rosidl_type_hash_t vision_msgs__msg__BoundingBox2D__EXPECTED_HASH = {1, {
    0x00, 0x8e, 0xac, 0xbb, 0x0c, 0xf8, 0xf2, 0x6e,
    0x83, 0x79, 0x55, 0x47, 0xbe, 0xd0, 0x13, 0xee,
    0xc3, 0x67, 0x54, 0x85, 0xef, 0x38, 0x6e, 0xc0,
    0x56, 0xd8, 0x0e, 0xaa, 0xf1, 0xc1, 0xce, 0x2f,
  }};
static const rosidl_type_hash_t vision_msgs__msg__Point2D__EXPECTED_HASH = {1, {
    0xea, 0xb0, 0xe8, 0x3f, 0x44, 0xab, 0x4d, 0xe9,
    0x2c, 0xea, 0xf7, 0x6d, 0xd7, 0xd0, 0xe7, 0x63,
    0x8f, 0x2b, 0xdd, 0x1d, 0x16, 0xff, 0xbe, 0x16,
    0xc7, 0xa1, 0x36, 0xc4, 0x8e, 0x40, 0x72, 0x47,
  }};
static const rosidl_type_hash_t vision_msgs__msg__Pose2D__EXPECTED_HASH = {1, {
    0xb0, 0xb5, 0xd1, 0xd7, 0xb4, 0xc2, 0x0d, 0xd4,
    0xfc, 0xde, 0x3e, 0xd1, 0xd4, 0xb2, 0x89, 0xa9,
    0x19, 0x29, 0x05, 0x72, 0x00, 0x7d, 0x67, 0xa9,
    0x70, 0x43, 0x81, 0x4d, 0xb2, 0x5d, 0x36, 0x48,
  }};
#endif

static char pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__TYPE_NAME[] = "pinkk_usb_insertion_interfaces/msg/UsbPortDetectionArray";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__TYPE_NAME[] = "pinkk_usb_insertion_interfaces/msg/Keypoint2D";
static char pinkk_usb_insertion_interfaces__msg__UsbPortDetection__TYPE_NAME[] = "pinkk_usb_insertion_interfaces/msg/UsbPortDetection";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char vision_msgs__msg__BoundingBox2D__TYPE_NAME[] = "vision_msgs/msg/BoundingBox2D";
static char vision_msgs__msg__Point2D__TYPE_NAME[] = "vision_msgs/msg/Point2D";
static char vision_msgs__msg__Pose2D__TYPE_NAME[] = "vision_msgs/msg/Pose2D";

// Define type names, field names, and default values
static char pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__FIELD_NAME__header[] = "header";
static char pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__FIELD_NAME__detections[] = "detections";

static rosidl_runtime_c__type_description__Field pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__FIELDS[] = {
  {
    {pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__FIELD_NAME__detections, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {pinkk_usb_insertion_interfaces__msg__UsbPortDetection__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__UsbPortDetection__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {vision_msgs__msg__BoundingBox2D__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {vision_msgs__msg__Point2D__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {vision_msgs__msg__Pose2D__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__TYPE_NAME, 56, 56},
      {pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__FIELDS, 2, 2},
    },
    {pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&pinkk_usb_insertion_interfaces__msg__Keypoint2D__EXPECTED_HASH, pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&pinkk_usb_insertion_interfaces__msg__UsbPortDetection__EXPECTED_HASH, pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&vision_msgs__msg__BoundingBox2D__EXPECTED_HASH, vision_msgs__msg__BoundingBox2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = vision_msgs__msg__BoundingBox2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&vision_msgs__msg__Point2D__EXPECTED_HASH, vision_msgs__msg__Point2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = vision_msgs__msg__Point2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&vision_msgs__msg__Pose2D__EXPECTED_HASH, vision_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = vision_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xea\\xb0\\x99\\xec\\x9d\\x80 \\xec\\x98\\x81\\xec\\x83\\x81 \\xed\\x94\\x84\\xeb\\xa0\\x88\\xec\\x9e\\x84\\xec\\x97\\x90\\xec\\x84\\x9c \\xea\\xb2\\x80\\xec\\xb6\\x9c\\xeb\\x90\\x9c USB-A \\xed\\x8f\\xac\\xed\\x8a\\xb8 \\xed\\x9b\\x84\\xeb\\xb3\\xb4 \\xeb\\xaa\\xa9\\xeb\\xa1\\x9d.\n"
  "std_msgs/Header header\n"
  "pinkk_usb_insertion_interfaces/UsbPortDetection[] detections";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__TYPE_NAME, 56, 56},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 118, 118},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_individual_type_description_source(NULL);
    sources[3] = *pinkk_usb_insertion_interfaces__msg__UsbPortDetection__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[5] = *vision_msgs__msg__BoundingBox2D__get_individual_type_description_source(NULL);
    sources[6] = *vision_msgs__msg__Point2D__get_individual_type_description_source(NULL);
    sources[7] = *vision_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
