; Auto-generated. Do not edit!


(cl:in-package volksbot-srv)


;//! \htmlinclude velocities-request.msg.html

(cl:defclass <velocities-request> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass velocities-request (<velocities-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocities-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocities-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name volksbot-srv:<velocities-request> is deprecated: use volksbot-srv:velocities-request instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <velocities-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader volksbot-srv:left-val is deprecated.  Use volksbot-srv:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <velocities-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader volksbot-srv:right-val is deprecated.  Use volksbot-srv:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocities-request>) ostream)
  "Serializes a message object of type '<velocities-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocities-request>) istream)
  "Deserializes a message object of type '<velocities-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocities-request>)))
  "Returns string type for a service object of type '<velocities-request>"
  "volksbot/velocitiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocities-request)))
  "Returns string type for a service object of type 'velocities-request"
  "volksbot/velocitiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocities-request>)))
  "Returns md5sum for a message object of type '<velocities-request>"
  "50c2436c38cded221d061b57126c4e40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocities-request)))
  "Returns md5sum for a message object of type 'velocities-request"
  "50c2436c38cded221d061b57126c4e40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocities-request>)))
  "Returns full string definition for message of type '<velocities-request>"
  (cl:format cl:nil "float64 left~%float64 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocities-request)))
  "Returns full string definition for message of type 'velocities-request"
  (cl:format cl:nil "float64 left~%float64 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocities-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocities-request>))
  "Converts a ROS message object to a list"
  (cl:list 'velocities-request
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
;//! \htmlinclude velocities-response.msg.html

(cl:defclass <velocities-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass velocities-response (<velocities-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocities-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocities-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name volksbot-srv:<velocities-response> is deprecated: use volksbot-srv:velocities-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocities-response>) ostream)
  "Serializes a message object of type '<velocities-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocities-response>) istream)
  "Deserializes a message object of type '<velocities-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocities-response>)))
  "Returns string type for a service object of type '<velocities-response>"
  "volksbot/velocitiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocities-response)))
  "Returns string type for a service object of type 'velocities-response"
  "volksbot/velocitiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocities-response>)))
  "Returns md5sum for a message object of type '<velocities-response>"
  "50c2436c38cded221d061b57126c4e40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocities-response)))
  "Returns md5sum for a message object of type 'velocities-response"
  "50c2436c38cded221d061b57126c4e40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocities-response>)))
  "Returns full string definition for message of type '<velocities-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocities-response)))
  "Returns full string definition for message of type 'velocities-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocities-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocities-response>))
  "Converts a ROS message object to a list"
  (cl:list 'velocities-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'velocities)))
  'velocities-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'velocities)))
  'velocities-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocities)))
  "Returns string type for a service object of type '<velocities>"
  "volksbot/velocities")