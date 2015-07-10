"""autogenerated by genpy from croc_Pose3D/Pose3D.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class Pose3D(genpy.Message):
  _md5sum = "78a9251aa2bab1e13b97b5f5d91aa653"
  _type = "croc_Pose3D/Pose3D"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

float32 x
float32 y
float32 z

float32 pitch
float32 roll
float32 yaw

geometry_msgs/Vector3 linear_acceleration

geometry_msgs/Vector3 calculated_linear_velocity



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
  __slots__ = ['header','x','y','z','pitch','roll','yaw','linear_acceleration','calculated_linear_velocity']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','float32','float32','geometry_msgs/Vector3','geometry_msgs/Vector3']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,x,y,z,pitch,roll,yaw,linear_acceleration,calculated_linear_velocity

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Pose3D, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.roll is None:
        self.roll = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.calculated_linear_velocity is None:
        self.calculated_linear_velocity = geometry_msgs.msg.Vector3()
    else:
      self.header = std_msgs.msg.Header()
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.pitch = 0.
      self.roll = 0.
      self.yaw = 0.
      self.linear_acceleration = geometry_msgs.msg.Vector3()
      self.calculated_linear_velocity = geometry_msgs.msg.Vector3()

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
      buff.write(_struct_6f6d.pack(_x.x, _x.y, _x.z, _x.pitch, _x.roll, _x.yaw, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.calculated_linear_velocity.x, _x.calculated_linear_velocity.y, _x.calculated_linear_velocity.z))
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
      if self.calculated_linear_velocity is None:
        self.calculated_linear_velocity = geometry_msgs.msg.Vector3()
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
      end += 72
      (_x.x, _x.y, _x.z, _x.pitch, _x.roll, _x.yaw, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.calculated_linear_velocity.x, _x.calculated_linear_velocity.y, _x.calculated_linear_velocity.z,) = _struct_6f6d.unpack(str[start:end])
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
      buff.write(_struct_6f6d.pack(_x.x, _x.y, _x.z, _x.pitch, _x.roll, _x.yaw, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.calculated_linear_velocity.x, _x.calculated_linear_velocity.y, _x.calculated_linear_velocity.z))
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
      if self.calculated_linear_velocity is None:
        self.calculated_linear_velocity = geometry_msgs.msg.Vector3()
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
      end += 72
      (_x.x, _x.y, _x.z, _x.pitch, _x.roll, _x.yaw, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.calculated_linear_velocity.x, _x.calculated_linear_velocity.y, _x.calculated_linear_velocity.z,) = _struct_6f6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_6f6d = struct.Struct("<6f6d")