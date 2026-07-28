# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UsbPortDetection(type):
    """Metaclass of message 'UsbPortDetection'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pinkk_usb_insertion_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pinkk_usb_insertion_interfaces.msg.UsbPortDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__usb_port_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__usb_port_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__usb_port_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__usb_port_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__usb_port_detection

            from pinkk_usb_insertion_interfaces.msg import Keypoint2D
            if Keypoint2D.__class__._TYPE_SUPPORT is None:
                Keypoint2D.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from vision_msgs.msg import BoundingBox2D
            if BoundingBox2D.__class__._TYPE_SUPPORT is None:
                BoundingBox2D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UsbPortDetection(metaclass=Metaclass_UsbPortDetection):
    """Message class 'UsbPortDetection'."""

    __slots__ = [
        '_header',
        '_detection_id',
        '_class_name',
        '_object_confidence',
        '_bbox',
        '_source_image_width',
        '_source_image_height',
        '_keypoints',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'detection_id': 'string',
        'class_name': 'string',
        'object_confidence': 'float',
        'bbox': 'vision_msgs/BoundingBox2D',
        'source_image_width': 'uint32',
        'source_image_height': 'uint32',
        'keypoints': 'pinkk_usb_insertion_interfaces/Keypoint2D[4]',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['vision_msgs', 'msg'], 'BoundingBox2D'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['pinkk_usb_insertion_interfaces', 'msg'], 'Keypoint2D'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.detection_id = kwargs.get('detection_id', str())
        self.class_name = kwargs.get('class_name', str())
        self.object_confidence = kwargs.get('object_confidence', float())
        from vision_msgs.msg import BoundingBox2D
        self.bbox = kwargs.get('bbox', BoundingBox2D())
        self.source_image_width = kwargs.get('source_image_width', int())
        self.source_image_height = kwargs.get('source_image_height', int())
        from pinkk_usb_insertion_interfaces.msg import Keypoint2D
        self.keypoints = kwargs.get(
            'keypoints',
            [Keypoint2D() for x in range(4)]
        )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.detection_id != other.detection_id:
            return False
        if self.class_name != other.class_name:
            return False
        if self.object_confidence != other.object_confidence:
            return False
        if self.bbox != other.bbox:
            return False
        if self.source_image_width != other.source_image_width:
            return False
        if self.source_image_height != other.source_image_height:
            return False
        if self.keypoints != other.keypoints:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def detection_id(self):
        """Message field 'detection_id'."""
        return self._detection_id

    @detection_id.setter
    def detection_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'detection_id' field must be of type 'str'"
        self._detection_id = value

    @builtins.property
    def class_name(self):
        """Message field 'class_name'."""
        return self._class_name

    @class_name.setter
    def class_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'class_name' field must be of type 'str'"
        self._class_name = value

    @builtins.property
    def object_confidence(self):
        """Message field 'object_confidence'."""
        return self._object_confidence

    @object_confidence.setter
    def object_confidence(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'object_confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'object_confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._object_confidence = value

    @builtins.property
    def bbox(self):
        """Message field 'bbox'."""
        return self._bbox

    @bbox.setter
    def bbox(self, value):
        if self._check_fields:
            from vision_msgs.msg import BoundingBox2D
            assert \
                isinstance(value, BoundingBox2D), \
                "The 'bbox' field must be a sub message of type 'BoundingBox2D'"
        self._bbox = value

    @builtins.property
    def source_image_width(self):
        """Message field 'source_image_width'."""
        return self._source_image_width

    @source_image_width.setter
    def source_image_width(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'source_image_width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'source_image_width' field must be an unsigned integer in [0, 4294967295]"
        self._source_image_width = value

    @builtins.property
    def source_image_height(self):
        """Message field 'source_image_height'."""
        return self._source_image_height

    @source_image_height.setter
    def source_image_height(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'source_image_height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'source_image_height' field must be an unsigned integer in [0, 4294967295]"
        self._source_image_height = value

    @builtins.property
    def keypoints(self):
        """Message field 'keypoints'."""
        return self._keypoints

    @keypoints.setter
    def keypoints(self, value):
        if self._check_fields:
            from pinkk_usb_insertion_interfaces.msg import Keypoint2D
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, Keypoint2D) for v in value) and
                 True), \
                "The 'keypoints' field must be a set or sequence with length 4 and each value of type 'Keypoint2D'"
        self._keypoints = value
