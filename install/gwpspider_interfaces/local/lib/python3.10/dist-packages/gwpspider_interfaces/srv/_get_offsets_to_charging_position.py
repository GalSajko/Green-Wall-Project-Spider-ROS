# generated from rosidl_generator_py/resource/_idl.py.em
# with input from gwpspider_interfaces:srv/GetOffsetsToChargingPosition.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetOffsetsToChargingPosition_Request(type):
    """Metaclass of message 'GetOffsetsToChargingPosition_Request'."""

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
            module = import_type_support('gwpspider_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'gwpspider_interfaces.srv.GetOffsetsToChargingPosition_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_offsets_to_charging_position__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_offsets_to_charging_position__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_offsets_to_charging_position__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_offsets_to_charging_position__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_offsets_to_charging_position__request

            from std_msgs.msg import Int16MultiArray
            if Int16MultiArray.__class__._TYPE_SUPPORT is None:
                Int16MultiArray.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetOffsetsToChargingPosition_Request(metaclass=Metaclass_GetOffsetsToChargingPosition_Request):
    """Message class 'GetOffsetsToChargingPosition_Request'."""

    __slots__ = [
        '_current_pins',
    ]

    _fields_and_field_types = {
        'current_pins': 'std_msgs/Int16MultiArray',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Int16MultiArray'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Int16MultiArray
        self.current_pins = kwargs.get('current_pins', Int16MultiArray())

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
        if self.current_pins != other.current_pins:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_pins(self):
        """Message field 'current_pins'."""
        return self._current_pins

    @current_pins.setter
    def current_pins(self, value):
        if __debug__:
            from std_msgs.msg import Int16MultiArray
            assert \
                isinstance(value, Int16MultiArray), \
                "The 'current_pins' field must be a sub message of type 'Int16MultiArray'"
        self._current_pins = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetOffsetsToChargingPosition_Response(type):
    """Metaclass of message 'GetOffsetsToChargingPosition_Response'."""

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
            module = import_type_support('gwpspider_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'gwpspider_interfaces.srv.GetOffsetsToChargingPosition_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_offsets_to_charging_position__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_offsets_to_charging_position__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_offsets_to_charging_position__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_offsets_to_charging_position__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_offsets_to_charging_position__response

            from std_msgs.msg import Float32MultiArray
            if Float32MultiArray.__class__._TYPE_SUPPORT is None:
                Float32MultiArray.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetOffsetsToChargingPosition_Response(metaclass=Metaclass_GetOffsetsToChargingPosition_Response):
    """Message class 'GetOffsetsToChargingPosition_Response'."""

    __slots__ = [
        '_offsets',
    ]

    _fields_and_field_types = {
        'offsets': 'std_msgs/Float32MultiArray',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Float32MultiArray'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Float32MultiArray
        self.offsets = kwargs.get('offsets', Float32MultiArray())

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
        if self.offsets != other.offsets:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def offsets(self):
        """Message field 'offsets'."""
        return self._offsets

    @offsets.setter
    def offsets(self, value):
        if __debug__:
            from std_msgs.msg import Float32MultiArray
            assert \
                isinstance(value, Float32MultiArray), \
                "The 'offsets' field must be a sub message of type 'Float32MultiArray'"
        self._offsets = value


class Metaclass_GetOffsetsToChargingPosition(type):
    """Metaclass of service 'GetOffsetsToChargingPosition'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('gwpspider_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'gwpspider_interfaces.srv.GetOffsetsToChargingPosition')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_offsets_to_charging_position

            from gwpspider_interfaces.srv import _get_offsets_to_charging_position
            if _get_offsets_to_charging_position.Metaclass_GetOffsetsToChargingPosition_Request._TYPE_SUPPORT is None:
                _get_offsets_to_charging_position.Metaclass_GetOffsetsToChargingPosition_Request.__import_type_support__()
            if _get_offsets_to_charging_position.Metaclass_GetOffsetsToChargingPosition_Response._TYPE_SUPPORT is None:
                _get_offsets_to_charging_position.Metaclass_GetOffsetsToChargingPosition_Response.__import_type_support__()


class GetOffsetsToChargingPosition(metaclass=Metaclass_GetOffsetsToChargingPosition):
    from gwpspider_interfaces.srv._get_offsets_to_charging_position import GetOffsetsToChargingPosition_Request as Request
    from gwpspider_interfaces.srv._get_offsets_to_charging_position import GetOffsetsToChargingPosition_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
