"""autogenerated by genpy from cyphy_serial_driver/mikoImu.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class mikoImu(genpy.Message):
  _md5sum = "76691c6d34f503f607812cc4761ad3a0"
  _type = "cyphy_serial_driver/mikoImu"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# This is a message to hold data from an Mikrokopter IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational angle should be in rad
#
# by Inkyu

Header header

float64 anglePitch
float64 angleRoll
float64 angleYaw
int32 stick_throttle
int32 barome_height
int32 batt
int32 extern_on

geometry_msgs/Vector3 linear_acceleration

int32[32] debugData


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['header','anglePitch','angleRoll','angleYaw','stick_throttle','barome_height','batt','extern_on','linear_acceleration','debugData']
  _slot_types = ['std_msgs/Header','float64','float64','float64','int32','int32','int32','int32','geometry_msgs/Vector3','int32[32]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,anglePitch,angleRoll,angleYaw,stick_throttle,barome_height,batt,extern_on,linear_acceleration,debugData

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(mikoImu, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.anglePitch is None:
        self.anglePitch = 0.
      if self.angleRoll is None:
        self.angleRoll = 0.
      if self.angleYaw is None:
        self.angleYaw = 0.
      if self.stick_throttle is None:
        self.stick_throttle = 0
      if self.barome_height is None:
        self.barome_height = 0
      if self.batt is None:
        self.batt = 0
      if self.extern_on is None:
        self.extern_on = 0
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.debugData is None:
        self.debugData = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    else:
      self.header = std_msgs.msg.Header()
      self.anglePitch = 0.
      self.angleRoll = 0.
      self.angleYaw = 0.
      self.stick_throttle = 0
      self.barome_height = 0
      self.batt = 0
      self.extern_on = 0
      self.linear_acceleration = geometry_msgs.msg.Vector3()
      self.debugData = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d4i3d.pack(_x.anglePitch, _x.angleRoll, _x.angleYaw, _x.stick_throttle, _x.barome_height, _x.batt, _x.extern_on, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z))
      buff.write(_struct_32i.pack(*self.debugData))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 64
      (_x.anglePitch, _x.angleRoll, _x.angleYaw, _x.stick_throttle, _x.barome_height, _x.batt, _x.extern_on, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z,) = _struct_3d4i3d.unpack(str[start:end])
      start = end
      end += 128
      self.debugData = _struct_32i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d4i3d.pack(_x.anglePitch, _x.angleRoll, _x.angleYaw, _x.stick_throttle, _x.barome_height, _x.batt, _x.extern_on, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z))
      buff.write(self.debugData.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 64
      (_x.anglePitch, _x.angleRoll, _x.angleYaw, _x.stick_throttle, _x.barome_height, _x.batt, _x.extern_on, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z,) = _struct_3d4i3d.unpack(str[start:end])
      start = end
      end += 128
      self.debugData = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=32)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_3d4i3d = struct.Struct("<3d4i3d")
_struct_32i = struct.Struct("<32i")
