; Auto-generated. Do not edit!


(cl:in-package riegl-msg)


;//! \htmlinclude RieglTime.msg.html

(cl:defclass <RieglTime> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass RieglTime (<RieglTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RieglTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RieglTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-msg:<RieglTime> is deprecated: use riegl-msg:RieglTime instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RieglTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-msg:header-val is deprecated.  Use riegl-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <RieglTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-msg:time-val is deprecated.  Use riegl-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RieglTime>) ostream)
  "Serializes a message object of type '<RieglTime>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RieglTime>) istream)
  "Deserializes a message object of type '<RieglTime>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RieglTime>)))
  "Returns string type for a message object of type '<RieglTime>"
  "riegl/RieglTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RieglTime)))
  "Returns string type for a message object of type 'RieglTime"
  "riegl/RieglTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RieglTime>)))
  "Returns md5sum for a message object of type '<RieglTime>"
  "dce2d1e2213c6c29642ee1125033b12b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RieglTime)))
  "Returns md5sum for a message object of type 'RieglTime"
  "dce2d1e2213c6c29642ee1125033b12b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RieglTime>)))
  "Returns full string definition for message of type '<RieglTime>"
  (cl:format cl:nil "Header header~%# 0 scanning~%# 1 frame start~%# 2 frame stop~%float64 time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RieglTime)))
  "Returns full string definition for message of type 'RieglTime"
  (cl:format cl:nil "Header header~%# 0 scanning~%# 1 frame start~%# 2 frame stop~%float64 time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RieglTime>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RieglTime>))
  "Converts a ROS message object to a list"
  (cl:list 'RieglTime
    (cl:cons ':header (header msg))
    (cl:cons ':time (time msg))
))
