// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool vision_msgs__msg__bounding_box2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * vision_msgs__msg__bounding_box2_d__convert_to_py(void * raw_ros_message);
bool pinkk_usb_insertion_interfaces__msg__keypoint2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * pinkk_usb_insertion_interfaces__msg__keypoint2_d__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool pinkk_usb_insertion_interfaces__msg__usb_port_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[72];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("pinkk_usb_insertion_interfaces.msg._usb_port_detection.UsbPortDetection", full_classname_dest, 71) == 0);
  }
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // detection_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "detection_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->detection_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // class_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "class_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->class_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // object_confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "object_confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->object_confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bbox
    PyObject * field = PyObject_GetAttrString(_pymsg, "bbox");
    if (!field) {
      return false;
    }
    if (!vision_msgs__msg__bounding_box2_d__convert_from_py(field, &ros_message->bbox)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // source_image_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "source_image_width");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->source_image_width = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // source_image_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "source_image_height");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->source_image_height = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // keypoints
    PyObject * field = PyObject_GetAttrString(_pymsg, "keypoints");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'keypoints'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = 4;
    pinkk_usb_insertion_interfaces__msg__Keypoint2D * dest = ros_message->keypoints;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!pinkk_usb_insertion_interfaces__msg__keypoint2_d__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * pinkk_usb_insertion_interfaces__msg__usb_port_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UsbPortDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("pinkk_usb_insertion_interfaces.msg._usb_port_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UsbPortDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * ros_message = (pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // detection_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->detection_id.data,
      strlen(ros_message->detection_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "detection_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // class_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->class_name.data,
      strlen(ros_message->class_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "class_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // object_confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->object_confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "object_confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bbox
    PyObject * field = NULL;
    field = vision_msgs__msg__bounding_box2_d__convert_to_py(&ros_message->bbox);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bbox", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // source_image_width
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->source_image_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "source_image_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // source_image_height
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->source_image_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "source_image_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // keypoints
    PyObject * field = NULL;
    size_t size = 4;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    pinkk_usb_insertion_interfaces__msg__Keypoint2D * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->keypoints[i]);
      PyObject * pyitem = pinkk_usb_insertion_interfaces__msg__keypoint2_d__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "keypoints", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
