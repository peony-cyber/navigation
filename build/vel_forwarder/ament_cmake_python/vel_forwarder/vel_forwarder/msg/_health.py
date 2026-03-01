# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vel_forwarder:msg/Health.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Health(type):
    """Metaclass of message 'Health'."""

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
                'vel_forwarder.msg.Health')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__health
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__health
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__health
            cls._TYPE_SUPPORT = module.type_support_msg__msg__health
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__health

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Health(metaclass=Metaclass_Health):
    """Message class 'Health'."""

    __slots__ = [
        '_hero',
        '_engineer',
        '_infantry1',
        '_infantry2',
        '_reserve',
        '_sentry',
    ]

    _fields_and_field_types = {
        'hero': 'uint16',
        'engineer': 'uint16',
        'infantry1': 'uint16',
        'infantry2': 'uint16',
        'reserve': 'uint16',
        'sentry': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.hero = kwargs.get('hero', int())
        self.engineer = kwargs.get('engineer', int())
        self.infantry1 = kwargs.get('infantry1', int())
        self.infantry2 = kwargs.get('infantry2', int())
        self.reserve = kwargs.get('reserve', int())
        self.sentry = kwargs.get('sentry', int())

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
        if self.hero != other.hero:
            return False
        if self.engineer != other.engineer:
            return False
        if self.infantry1 != other.infantry1:
            return False
        if self.infantry2 != other.infantry2:
            return False
        if self.reserve != other.reserve:
            return False
        if self.sentry != other.sentry:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def hero(self):
        """Message field 'hero'."""
        return self._hero

    @hero.setter
    def hero(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hero' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'hero' field must be an unsigned integer in [0, 65535]"
        self._hero = value

    @builtins.property
    def engineer(self):
        """Message field 'engineer'."""
        return self._engineer

    @engineer.setter
    def engineer(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'engineer' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'engineer' field must be an unsigned integer in [0, 65535]"
        self._engineer = value

    @builtins.property
    def infantry1(self):
        """Message field 'infantry1'."""
        return self._infantry1

    @infantry1.setter
    def infantry1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'infantry1' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'infantry1' field must be an unsigned integer in [0, 65535]"
        self._infantry1 = value

    @builtins.property
    def infantry2(self):
        """Message field 'infantry2'."""
        return self._infantry2

    @infantry2.setter
    def infantry2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'infantry2' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'infantry2' field must be an unsigned integer in [0, 65535]"
        self._infantry2 = value

    @builtins.property
    def reserve(self):
        """Message field 'reserve'."""
        return self._reserve

    @reserve.setter
    def reserve(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reserve' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'reserve' field must be an unsigned integer in [0, 65535]"
        self._reserve = value

    @builtins.property
    def sentry(self):
        """Message field 'sentry'."""
        return self._sentry

    @sentry.setter
    def sentry(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sentry' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'sentry' field must be an unsigned integer in [0, 65535]"
        self._sentry = value
