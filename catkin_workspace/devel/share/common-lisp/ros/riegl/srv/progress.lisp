; Auto-generated. Do not edit!


(cl:in-package riegl-srv)


;//! \htmlinclude progress-request.msg.html

(cl:defclass <progress-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass progress-request (<progress-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <progress-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'progress-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<progress-request> is deprecated: use riegl-srv:progress-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <progress-request>) ostream)
  "Serializes a message object of type '<progress-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <progress-request>) istream)
  "Deserializes a message object of type '<progress-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<progress-request>)))
  "Returns string type for a service object of type '<progress-request>"
  "riegl/progressRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'progress-request)))
  "Returns string type for a service object of type 'progress-request"
  "riegl/progressRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<progress-request>)))
  "Returns md5sum for a message object of type '<progress-request>"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'progress-request)))
  "Returns md5sum for a message object of type 'progress-request"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<progress-request>)))
  "Returns full string definition for message of type '<progress-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'progress-request)))
  "Returns full string definition for message of type 'progress-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <progress-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <progress-request>))
  "Converts a ROS message object to a list"
  (cl:list 'progress-request
))
;//! \htmlinclude progress-response.msg.html

(cl:defclass <progress-response> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0))
)

(cl:defclass progress-response (<progress-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <progress-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'progress-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<progress-response> is deprecated: use riegl-srv:progress-response instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <progress-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:progress-val is deprecated.  Use riegl-srv:progress instead.")
  (progress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <progress-response>) ostream)
  "Serializes a message object of type '<progress-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <progress-response>) istream)
  "Deserializes a message object of type '<progress-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<progress-response>)))
  "Returns string type for a service object of type '<progress-response>"
  "riegl/progressResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'progress-response)))
  "Returns string type for a service object of type 'progress-response"
  "riegl/progressResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<progress-response>)))
  "Returns md5sum for a message object of type '<progress-response>"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'progress-response)))
  "Returns md5sum for a message object of type 'progress-response"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<progress-response>)))
  "Returns full string definition for message of type '<progress-response>"
  (cl:format cl:nil "float64 progress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'progress-response)))
  "Returns full string definition for message of type 'progress-response"
  (cl:format cl:nil "float64 progress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <progress-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <progress-response>))
  "Converts a ROS message object to a list"
  (cl:list 'progress-response
    (cl:cons ':progress (progress msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'progress)))
  'progress-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'progress)))
  'progress-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'progress)))
  "Returns string type for a service object of type '<progress>"
  "riegl/progress")