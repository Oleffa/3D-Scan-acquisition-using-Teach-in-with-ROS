; Auto-generated. Do not edit!


(cl:in-package riegl-msg)


;//! \htmlinclude RieglStatus.msg.html

(cl:defclass <RieglStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass RieglStatus (<RieglStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RieglStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RieglStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-msg:<RieglStatus> is deprecated: use riegl-msg:RieglStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RieglStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-msg:header-val is deprecated.  Use riegl-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <RieglStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-msg:status-val is deprecated.  Use riegl-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RieglStatus>) ostream)
  "Serializes a message object of type '<RieglStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RieglStatus>) istream)
  "Deserializes a message object of type '<RieglStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RieglStatus>)))
  "Returns string type for a message object of type '<RieglStatus>"
  "riegl/RieglStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RieglStatus)))
  "Returns string type for a message object of type 'RieglStatus"
  "riegl/RieglStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RieglStatus>)))
  "Returns md5sum for a message object of type '<RieglStatus>"
  "464017b6d3bfd385079f3c7dc3b2e6be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RieglStatus)))
  "Returns md5sum for a message object of type 'RieglStatus"
  "464017b6d3bfd385079f3c7dc3b2e6be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RieglStatus>)))
  "Returns full string definition for message of type '<RieglStatus>"
  (cl:format cl:nil "Header header~%# 0 scanning~%# 1 frame start~%# 2 frame stop~%int32 status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RieglStatus)))
  "Returns full string definition for message of type 'RieglStatus"
  (cl:format cl:nil "Header header~%# 0 scanning~%# 1 frame start~%# 2 frame stop~%int32 status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RieglStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RieglStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'RieglStatus
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
))
