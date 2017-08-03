; Auto-generated. Do not edit!


(cl:in-package riegl-srv)


;//! \htmlinclude angle-request.msg.html

(cl:defclass <angle-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass angle-request (<angle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <angle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'angle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<angle-request> is deprecated: use riegl-srv:angle-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:angle-val is deprecated.  Use riegl-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <angle-request>) ostream)
  "Serializes a message object of type '<angle-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <angle-request>) istream)
  "Deserializes a message object of type '<angle-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<angle-request>)))
  "Returns string type for a service object of type '<angle-request>"
  "riegl/angleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'angle-request)))
  "Returns string type for a service object of type 'angle-request"
  "riegl/angleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<angle-request>)))
  "Returns md5sum for a message object of type '<angle-request>"
  "4edb55038e2b888976a0c0c56935341c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'angle-request)))
  "Returns md5sum for a message object of type 'angle-request"
  "4edb55038e2b888976a0c0c56935341c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<angle-request>)))
  "Returns full string definition for message of type '<angle-request>"
  (cl:format cl:nil "float64 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'angle-request)))
  "Returns full string definition for message of type 'angle-request"
  (cl:format cl:nil "float64 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <angle-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <angle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'angle-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude angle-response.msg.html

(cl:defclass <angle-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass angle-response (<angle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <angle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'angle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<angle-response> is deprecated: use riegl-srv:angle-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <angle-response>) ostream)
  "Serializes a message object of type '<angle-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <angle-response>) istream)
  "Deserializes a message object of type '<angle-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<angle-response>)))
  "Returns string type for a service object of type '<angle-response>"
  "riegl/angleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'angle-response)))
  "Returns string type for a service object of type 'angle-response"
  "riegl/angleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<angle-response>)))
  "Returns md5sum for a message object of type '<angle-response>"
  "4edb55038e2b888976a0c0c56935341c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'angle-response)))
  "Returns md5sum for a message object of type 'angle-response"
  "4edb55038e2b888976a0c0c56935341c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<angle-response>)))
  "Returns full string definition for message of type '<angle-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'angle-response)))
  "Returns full string definition for message of type 'angle-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <angle-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <angle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'angle-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'angle)))
  'angle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'angle)))
  'angle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'angle)))
  "Returns string type for a service object of type '<angle>"
  "riegl/angle")