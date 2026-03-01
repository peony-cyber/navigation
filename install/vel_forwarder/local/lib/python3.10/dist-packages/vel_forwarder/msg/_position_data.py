# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vel_forwarder:msg/PositionData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PositionData(type):
    """Metaclass of message 'PositionData'."""

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
            module = import_type_support('vel_forwarder')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vel_forwarder.msg.PositionData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__position_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__position_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__position_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__position_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__position_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PositionData(metaclass=Metaclass_PositionData):
    """Message class 'PositionData'."""

    __slots__ = [
        '_friendcarid',
        '_friendx',
        '_friendy',
        '_enemycarid',
        '_enemyx',
        '_enemyy',
    ]

    _fields_and_field_types = {
        'friendcarid': 'uint8',
        'friendx': 'int16',
        'friendy': 'int16',
        'enemycarid': 'uint8',
        'enemyx': 'int16',
        'enemyy': 'int16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.friendcarid = kwargs.get('friendcarid', int())
        self.friendx = kwargs.get('friendx', int())
        self.friendy = kwargs.get('friendy', int())
        self.enemycarid = kwargs.get('enemycarid', int())
        self.enemyx = kwargs.get('enemyx', int())
        self.enemyy = kwargs.get('enemyy', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.friendcarid != other.friendcarid:
            return False
        if self.friendx != other.friendx:
            return False
        if self.friendy != other.friendy:
            return False
        if self.enemycarid != other.enemycarid:
            return False
        if self.enemyx != other.enemyx:
            return False
        if self.enemyy != other.enemyy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def friendcarid(self):
        """Message field 'friendcarid'."""
        return self._friendcarid

    @friendcarid.setter
    def friendcarid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'friendcarid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'friendcarid' field must be an unsigned integer in [0, 255]"
        self._friendcarid = value

    @builtins.property
    def friendx(self):
        """Message field 'friendx'."""
        return self._friendx

    @friendx.setter
    def friendx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'friendx' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'friendx' field must be an integer in [-32768, 32767]"
        self._friendx = value

    @builtins.property
    def friendy(self):
        """Message field 'friendy'."""
        return self._friendy

    @friendy.setter
    def friendy(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'friendy' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'friendy' field must be an integer in [-32768, 32767]"
        self._friendy = value

    @builtins.property
    def enemycarid(self):
        """Message field 'enemycarid'."""
        return self._enemycarid

    @enemycarid.setter
    def enemycarid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemycarid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'enemycarid' field must be an unsigned integer in [0, 255]"
        self._enemycarid = value

    @builtins.property
    def enemyx(self):
        """Message field 'enemyx'."""
        return self._enemyx

    @enemyx.setter
    def enemyx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemyx' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'enemyx' field must be an integer in [-32768, 32767]"
        self._enemyx = value

    @builtins.property
    def enemyy(self):
        """Message field 'enemyy'."""
        return self._enemyy

    @enemyy.setter
    def enemyy(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemyy' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'enemyy' field must be an integer in [-32768, 32767]"
        self._enemyy = value
