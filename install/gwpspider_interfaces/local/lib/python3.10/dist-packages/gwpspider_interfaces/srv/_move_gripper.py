# generated from rosidl_generator_py/resource/_idl.py.em
# with input from gwpspider_interfaces:srv/MoveGripper.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MoveGripper_Request(type):
    """Metaclass of message 'MoveGripper_Request'."""

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
                'gwpspider_interfaces.srv.MoveGripper_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__move_gripper__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__move_gripper__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__move_gripper__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__move_gripper__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__move_gripper__request

            from gwpspider_interfaces.msg import GripperCommand
            if GripperCommand.__class__._TYPE_SUPPORT is None:
                GripperCommand.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveGripper_Request(metaclass=Metaclass_MoveGripper_Request):
    """Message class 'MoveGripper_Request'."""

    __slots__ = [
        '_instructions',
    ]

    _fields_and_field_types = {
        'instructions': 'gwpspider_interfaces/GripperCommand',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['gwpspider_interfaces', 'msg'], 'GripperCommand'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from gwpspider_interfaces.msg import GripperCommand
        self.instructions = kwargs.get('instructions', GripperCommand())

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
        if self.instructions != other.instructions:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def instructions(self):
        """Message field 'instructions'."""
        return self._instructions

    @instructions.setter
    def instructions(self, value):
        if __debug__:
            from gwpspider_interfaces.msg import GripperCommand
            assert \
                isinstance(value, GripperCommand), \
                "The 'instructions' field must be a sub message of type 'GripperCommand'"
        self._instructions = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveGripper_Response(type):
    """Metaclass of message 'MoveGripper_Response'."""

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
                'gwpspider_interfaces.srv.MoveGripper_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__move_gripper__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__move_gripper__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__move_gripper__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__move_gripper__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__move_gripper__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveGripper_Response(metaclass=Metaclass_MoveGripper_Response):
    """Message class 'MoveGripper_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_MoveGripper(type):
    """Metaclass of service 'MoveGripper'."""

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
                'gwpspider_interfaces.srv.MoveGripper')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__move_gripper

            from gwpspider_interfaces.srv import _move_gripper
            if _move_gripper.Metaclass_MoveGripper_Request._TYPE_SUPPORT is None:
                _move_gripper.Metaclass_MoveGripper_Request.__import_type_support__()
            if _move_gripper.Metaclass_MoveGripper_Response._TYPE_SUPPORT is None:
                _move_gripper.Metaclass_MoveGripper_Response.__import_type_support__()


class MoveGripper(metaclass=Metaclass_MoveGripper):
    from gwpspider_interfaces.srv._move_gripper import MoveGripper_Request as Request
    from gwpspider_interfaces.srv._move_gripper import MoveGripper_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
