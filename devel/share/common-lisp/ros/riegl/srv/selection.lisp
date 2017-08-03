; Auto-generated. Do not edit!


(cl:in-package riegl-srv)


;//! \htmlinclude selection-request.msg.html

(cl:defclass <selection-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass selection-request (<selection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <selection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'selection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<selection-request> is deprecated: use riegl-srv:selection-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <selection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:type-val is deprecated.  Use riegl-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <selection-request>) ostream)
  "Serializes a message object of type '<selection-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <selection-request>) istream)
  "Deserializes a message object of type '<selection-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<selection-request>)))
  "Returns string type for a service object of type '<selection-request>"
  "riegl/selectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'selection-request)))
  "Returns string type for a service object of type 'selection-request"
  "riegl/selectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<selection-request>)))
  "Returns md5sum for a message object of type '<selection-request>"
  "79260b638c2b88f6366fba8cd5157389")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'selection-request)))
  "Returns md5sum for a message object of type 'selection-request"
  "79260b638c2b88f6366fba8cd5157389")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<selection-request>)))
  "Returns full string definition for message of type '<selection-request>"
  (cl:format cl:nil "int64 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'selection-request)))
  "Returns full string definition for message of type 'selection-request"
  (cl:format cl:nil "int64 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <selection-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <selection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'selection-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude selection-response.msg.html

(cl:defclass <selection-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass selection-response (<selection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <selection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'selection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<selection-response> is deprecated: use riegl-srv:selection-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <selection-response>) ostream)
  "Serializes a message object of type '<selection-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <selection-response>) istream)
  "Deserializes a message object of type '<selection-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<selection-response>)))
  "Returns string type for a service object of type '<selection-response>"
  "riegl/selectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'selection-response)))
  "Returns string type for a service object of type 'selection-response"
  "riegl/selectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<selection-response>)))
  "Returns md5sum for a message object of type '<selection-response>"
  "79260b638c2b88f6366fba8cd5157389")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'selection-response)))
  "Returns md5sum for a message object of type 'selection-response"
  "79260b638c2b88f6366fba8cd5157389")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<selection-response>)))
  "Returns full string definition for message of type '<selection-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'selection-response)))
  "Returns full string definition for message of type 'selection-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <selection-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <selection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'selection-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'selection)))
  'selection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'selection)))
  'selection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'selection)))
  "Returns string type for a service object of type '<selection>"
  "riegl/selection")