# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tidal_vehicle_interfaces:msg/TerrainState.idl
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


class Metaclass_TerrainState(type):
    """Metaclass of message 'TerrainState'."""

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
            module = import_type_support('tidal_vehicle_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tidal_vehicle_interfaces.msg.TerrainState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__terrain_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__terrain_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__terrain_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__terrain_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__terrain_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TerrainState(metaclass=Metaclass_TerrainState):
    """Message class 'TerrainState'."""

    __slots__ = [
        '_header',
        '_tide_state',
        '_tide_risk',
        '_water_level_m',
        '_tide_rate_m_per_minute',
        '_seconds_until_corridor_unsafe',
        '_corridor_traversable',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'tide_state': 'string',
        'tide_risk': 'float',
        'water_level_m': 'float',
        'tide_rate_m_per_minute': 'float',
        'seconds_until_corridor_unsafe': 'float',
        'corridor_traversable': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.tide_state = kwargs.get('tide_state', str())
        self.tide_risk = kwargs.get('tide_risk', float())
        self.water_level_m = kwargs.get('water_level_m', float())
        self.tide_rate_m_per_minute = kwargs.get('tide_rate_m_per_minute', float())
        self.seconds_until_corridor_unsafe = kwargs.get('seconds_until_corridor_unsafe', float())
        self.corridor_traversable = kwargs.get('corridor_traversable', bool())

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
        if self.tide_state != other.tide_state:
            return False
        if self.tide_risk != other.tide_risk:
            return False
        if self.water_level_m != other.water_level_m:
            return False
        if self.tide_rate_m_per_minute != other.tide_rate_m_per_minute:
            return False
        if self.seconds_until_corridor_unsafe != other.seconds_until_corridor_unsafe:
            return False
        if self.corridor_traversable != other.corridor_traversable:
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
    def tide_state(self):
        """Message field 'tide_state'."""
        return self._tide_state

    @tide_state.setter
    def tide_state(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'tide_state' field must be of type 'str'"
        self._tide_state = value

    @builtins.property
    def tide_risk(self):
        """Message field 'tide_risk'."""
        return self._tide_risk

    @tide_risk.setter
    def tide_risk(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'tide_risk' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tide_risk' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tide_risk = value

    @builtins.property
    def water_level_m(self):
        """Message field 'water_level_m'."""
        return self._water_level_m

    @water_level_m.setter
    def water_level_m(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'water_level_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'water_level_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._water_level_m = value

    @builtins.property
    def tide_rate_m_per_minute(self):
        """Message field 'tide_rate_m_per_minute'."""
        return self._tide_rate_m_per_minute

    @tide_rate_m_per_minute.setter
    def tide_rate_m_per_minute(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'tide_rate_m_per_minute' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tide_rate_m_per_minute' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tide_rate_m_per_minute = value

    @builtins.property
    def seconds_until_corridor_unsafe(self):
        """Message field 'seconds_until_corridor_unsafe'."""
        return self._seconds_until_corridor_unsafe

    @seconds_until_corridor_unsafe.setter
    def seconds_until_corridor_unsafe(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'seconds_until_corridor_unsafe' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'seconds_until_corridor_unsafe' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._seconds_until_corridor_unsafe = value

    @builtins.property
    def corridor_traversable(self):
        """Message field 'corridor_traversable'."""
        return self._corridor_traversable

    @corridor_traversable.setter
    def corridor_traversable(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'corridor_traversable' field must be of type 'bool'"
        self._corridor_traversable = value
