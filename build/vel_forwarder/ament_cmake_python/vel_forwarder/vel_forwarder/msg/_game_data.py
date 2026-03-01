# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vel_forwarder:msg/GameData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GameData(type):
    """Metaclass of message 'GameData'."""

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
                'vel_forwarder.msg.GameData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__game_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__game_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__game_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__game_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__game_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GameData(metaclass=Metaclass_GameData):
    """Message class 'GameData'."""

    __slots__ = [
        '_gamecode',
        '_ammoleft',
        '_timeleft',
        '_selfhealth',
        '_exteventdata',
    ]

    _fields_and_field_types = {
        'gamecode': 'uint16',
        'ammoleft': 'uint16',
        'timeleft': 'uint16',
        'selfhealth': 'uint16',
        'exteventdata': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gamecode = kwargs.get('gamecode', int())
        self.ammoleft = kwargs.get('ammoleft', int())
        self.timeleft = kwargs.get('timeleft', int())
        self.selfhealth = kwargs.get('selfhealth', int())
        self.exteventdata = kwargs.get('exteventdata', int())

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
        if self.gamecode != other.gamecode:
            return False
        if self.ammoleft != other.ammoleft:
            return False
        if self.timeleft != other.timeleft:
            return False
        if self.selfhealth != other.selfhealth:
            return False
        if self.exteventdata != other.exteventdata:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def gamecode(self):
        """Message field 'gamecode'."""
        return self._gamecode

    @gamecode.setter
    def gamecode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gamecode' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'gamecode' field must be an unsigned integer in [0, 65535]"
        self._gamecode = value

    @builtins.property
    def ammoleft(self):
        """Message field 'ammoleft'."""
        return self._ammoleft

    @ammoleft.setter
    def ammoleft(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ammoleft' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'ammoleft' field must be an unsigned integer in [0, 65535]"
        self._ammoleft = value

    @builtins.property
    def timeleft(self):
        """Message field 'timeleft'."""
        return self._timeleft

    @timeleft.setter
    def timeleft(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timeleft' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'timeleft' field must be an unsigned integer in [0, 65535]"
        self._timeleft = value

    @builtins.property
    def selfhealth(self):
        """Message field 'selfhealth'."""
        return self._selfhealth

    @selfhealth.setter
    def selfhealth(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'selfhealth' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'selfhealth' field must be an unsigned integer in [0, 65535]"
        self._selfhealth = value

    @builtins.property
    def exteventdata(self):
        """Message field 'exteventdata'."""
        return self._exteventdata

    @exteventdata.setter
    def exteventdata(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'exteventdata' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'exteventdata' field must be an unsigned integer in [0, 4294967295]"
        self._exteventdata = value
