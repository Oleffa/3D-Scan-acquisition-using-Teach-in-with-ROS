; Auto-generated. Do not edit!


(cl:in-package riegl-srv)


;//! \htmlinclude inclination-request.msg.html

(cl:defclass <inclination-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass inclination-request (<inclination-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <inclination-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'inclination-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<inclination-request> is deprecated: use riegl-srv:inclination-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <inclination-request>) ostream)
  "Serializes a message object of type '<inclination-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <inclination-request>) istream)
  "Deserializes a message object of type '<inclination-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<inclination-request>)))
  "Returns string type for a service object of type '<inclination-request>"
  "riegl/inclinationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'inclination-request)))
  "Returns string type for a service object of type 'inclination-request"
  "riegl/inclinationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<inclination-request>)))
  "Returns md5sum for a message object of type '<inclination-request>"
  "0f5aa311af37faead3f9e90f420c9603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'inclination-request)))
  "Returns md5sum for a message object of type 'inclination-request"
  "0f5aa311af37faead3f9e90f420c9603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<inclination-request>)))
  "Returns full string definition for message of type '<inclination-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'inclination-request)))
  "Returns full string definition for message of type 'inclination-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <inclination-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <inclination-request>))
  "Converts a ROS message object to a list"
  (cl:list 'inclination-request
))
;//! \htmlinclude inclination-response.msg.html

(cl:defclass <inclination-response> (roslisp-msg-protocol:ros-message)
  ((roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0))
)

(cl:defclass inclination-response (<inclination-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <inclination-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'inclination-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<inclination-response> is deprecated: use riegl-srv:inclination-response instead.")))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <inclination-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:roll-val is deprecated.  Use riegl-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <inclination-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:pitch-val is deprecated.  Use riegl-srv:pitch instead.")
  (pitch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <inclination-response>) ostream)
  "Serializes a message object of type '<inclination-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <inclination-response>) istream)
  "Deserializes a message object of type '<inclination-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<inclination-response>)))
  "Returns string type for a service object of type '<inclination-response>"
  "riegl/inclinationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'inclination-response)))
  "Returns string type for a service object of type 'inclination-response"
  "riegl/inclinationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<inclination-response>)))
  "Returns md5sum for a message object of type '<inclination-response>"
  "0f5aa311af37faead3f9e90f420c9603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'inclination-response)))
  "Returns md5sum for a message object of type 'inclination-response"
  "0f5aa311af37faead3f9e90f420c9603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<inclination-response>)))
  "Returns full string definition for message of type '<inclination-response>"
  (cl:format cl:nil "float64 roll~%float64 pitch~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'inclination-response)))
  "Returns full string definition for message of type 'inclination-response"
  (cl:format cl:nil "float64 roll~%float64 pitch~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <inclination-response>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <inclination-response>))
  "Converts a ROS message object to a list"
  (cl:list 'inclination-response
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'inclination)))
  'inclination-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'inclination)))
  'inclination-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'inclination)))
  "Returns string type for a service object of type '<inclination>"
  "riegl/inclination")