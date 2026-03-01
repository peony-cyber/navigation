# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vel_forwarder:msg/BuffData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BuffData(type):
    """Metaclass of message 'BuffData'."""

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
                'vel_forwarder.msg.BuffData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__buff_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__buff_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__buff_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__buff_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__buff_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BuffData(metaclass=Metaclass_BuffData):
    """Message class 'BuffData'."""

    __slots__ = [
        '_recoverybuff',
        '_coolingbuff',
        '_defencebuff',
        '_vulnerabilitybuff',
        '_attackbuff',
        '_remainingenergy',
    ]

    _fields_and_field_types = {
        'recoverybuff': 'uint8',
        'coolingbuff': 'uint8',
        'defencebuff': 'uint8',
        'vulnerabilitybuff': 'uint8',
        'attackbuff': 'uint16',
        'remainingenergy': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.recoverybuff = kwargs.get('recoverybuff', int())
        self.coolingbuff = kwargs.get('coolingbuff', int())
        self.defencebuff = kwargs.get('defencebuff', int())
        self.vulnerabilitybuff = kwargs.get('vulnerabilitybuff', int())
        self.attackbuff = kwargs.get('attackbuff', int())
        self.remainingenergy = kwargs.get('remainingenergy', int())

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
        if self.recoverybuff != other.recoverybuff:
            return False
        if self.coolingbuff != other.coolingbuff:
            return False
        if self.defencebuff != other.defencebuff:
            return False
        if self.vulnerabilitybuff != other.vulnerabilitybuff:
            return False
        if self.attackbuff != other.attackbuff:
            return False
        if self.remainingenergy != other.remainingenergy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def recoverybuff(self):
        """Message field 'recoverybuff'."""
        return self._recoverybuff

    @recoverybuff.setter
    def recoverybuff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'recoverybuff' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'recoverybuff' field must be an unsigned integer in [0, 255]"
        self._recoverybuff = value

    @builtins.property
    def coolingbuff(self):
        """Message field 'coolingbuff'."""
        return self._coolingbuff

    @coolingbuff.setter
    def coolingbuff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'coolingbuff' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'coolingbuff' field must be an unsigned integer in [0, 255]"
        self._coolingbuff = value

    @builtins.property
    def defencebuff(self):
        """Message field 'defencebuff'."""
        return self._defencebuff

    @defencebuff.setter
    def defencebuff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'defencebuff' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'defencebuff' field must be an unsigned integer in [0, 255]"
        self._defencebuff = value

    @builtins.property
    def vulnerabilitybuff(self):
        """Message field 'vulnerabilitybuff'."""
        return self._vulnerabilitybuff

    @vulnerabilitybuff.setter
    def vulnerabilitybuff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vulnerabilitybuff' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'vulnerabilitybuff' field must be an unsigned integer in [0, 255]"
        self._vulnerabilitybuff = value

    @builtins.property
    def attackbuff(self):
        """Message field 'attackbuff'."""
        return self._attackbuff

    @attackbuff.setter
    def attackbuff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'attackbuff' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'attackbuff' field must be an unsigned integer in [0, 65535]"
        self._attackbuff = value

    @builtins.property
    def remainingenergy(self):
        """Message field 'remainingenergy'."""
        return self._remainingenergy

    @remainingenergy.setter
    def remainingenergy(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'remainingenergy' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'remainingenergy' field must be an unsigned integer in [0, 255]"
        self._remainingenergy = value
