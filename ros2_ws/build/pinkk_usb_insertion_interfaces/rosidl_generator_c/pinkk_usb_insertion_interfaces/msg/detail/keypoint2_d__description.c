// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pinkk_usb_insertion_interfaces:msg/Keypoint2D.idl
// generated code does not contain a copyright notice

#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_type_hash_t *
pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x64, 0x6b, 0x09, 0x1d, 0x0a, 0xea, 0x80, 0x8a,
      0x3a, 0x5d, 0x00, 0x84, 0xfa, 0xf2, 0x13, 0x7f,
      0x61, 0x3e, 0x49, 0xe0, 0xf9, 0x88, 0xc2, 0xa2,
      0x9e, 0x5f, 0x83, 0x57, 0xe2, 0xb4, 0x34, 0x30,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__TYPE_NAME[] = "pinkk_usb_insertion_interfaces/msg/Keypoint2D";

// Define type names, field names, and default values
static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__index[] = "index";
static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__x[] = "x";
static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__y[] = "y";
static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__confidence[] = "confidence";
static char pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__visible[] = "visible";

static rosidl_runtime_c__type_description__Field pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELDS[] = {
  {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__index, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELD_NAME__visible, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinkk_usb_insertion_interfaces__msg__Keypoint2D__TYPE_NAME, 45, 45},
      {pinkk_usb_insertion_interfaces__msg__Keypoint2D__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# YOLO\\xea\\xb0\\x80 \\xea\\xb2\\x80\\xec\\xb6\\x9c\\xed\\x95\\x9c \\xed\\x95\\x98\\xeb\\x82\\x98\\xec\\x9d\\x98 2D keypoint.\n"
  "uint8 index\n"
  "float64 x\n"
  "float64 y\n"
  "float32 confidence\n"
  "bool visible";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinkk_usb_insertion_interfaces__msg__Keypoint2D__TYPE_NAME, 45, 45},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 93, 93},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinkk_usb_insertion_interfaces__msg__Keypoint2D__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
