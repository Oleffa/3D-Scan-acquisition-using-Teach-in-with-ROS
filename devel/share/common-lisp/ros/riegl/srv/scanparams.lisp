; Auto-generated. Do not edit!


(cl:in-package riegl-srv)


;//! \htmlinclude scanparams-request.msg.html

(cl:defclass <scanparams-request> (roslisp-msg-protocol:ros-message)
  ((lineangle
    :reader lineangle
    :initarg :lineangle
    :type cl:float
    :initform 0.0)
   (frameangle
    :reader frameangle
    :initarg :frameangle
    :type cl:float
    :initform 0.0)
   (maxlineangle
    :reader maxlineangle
    :initarg :maxlineangle
    :type cl:float
    :initform 0.0)
   (minlineangle
    :reader minlineangle
    :initarg :minlineangle
    :type cl:float
    :initform 0.0)
   (maxframeangle
    :reader maxframeangle
    :initarg :maxframeangle
    :type cl:float
    :initform 0.0)
   (minframeangle
    :reader minframeangle
    :initarg :minframeangle
    :type cl:float
    :initform 0.0))
)

(cl:defclass scanparams-request (<scanparams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scanparams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scanparams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<scanparams-request> is deprecated: use riegl-srv:scanparams-request instead.")))

(cl:ensure-generic-function 'lineangle-val :lambda-list '(m))
(cl:defmethod lineangle-val ((m <scanparams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:lineangle-val is deprecated.  Use riegl-srv:lineangle instead.")
  (lineangle m))

(cl:ensure-generic-function 'frameangle-val :lambda-list '(m))
(cl:defmethod frameangle-val ((m <scanparams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:frameangle-val is deprecated.  Use riegl-srv:frameangle instead.")
  (frameangle m))

(cl:ensure-generic-function 'maxlineangle-val :lambda-list '(m))
(cl:defmethod maxlineangle-val ((m <scanparams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:maxlineangle-val is deprecated.  Use riegl-srv:maxlineangle instead.")
  (maxlineangle m))

(cl:ensure-generic-function 'minlineangle-val :lambda-list '(m))
(cl:defmethod minlineangle-val ((m <scanparams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:minlineangle-val is deprecated.  Use riegl-srv:minlineangle instead.")
  (minlineangle m))

(cl:ensure-generic-function 'maxframeangle-val :lambda-list '(m))
(cl:defmethod maxframeangle-val ((m <scanparams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:maxframeangle-val is deprecated.  Use riegl-srv:maxframeangle instead.")
  (maxframeangle m))

(cl:ensure-generic-function 'minframeangle-val :lambda-list '(m))
(cl:defmethod minframeangle-val ((m <scanparams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader riegl-srv:minframeangle-val is deprecated.  Use riegl-srv:minframeangle instead.")
  (minframeangle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scanparams-request>) ostream)
  "Serializes a message object of type '<scanparams-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lineangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frameangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxlineangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minlineangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxframeangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minframeangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scanparams-request>) istream)
  "Deserializes a message object of type '<scanparams-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lineangle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frameangle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxlineangle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minlineangle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxframeangle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minframeangle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scanparams-request>)))
  "Returns string type for a service object of type '<scanparams-request>"
  "riegl/scanparamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanparams-request)))
  "Returns string type for a service object of type 'scanparams-request"
  "riegl/scanparamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scanparams-request>)))
  "Returns md5sum for a message object of type '<scanparams-request>"
  "0852ff892d6a7a08b5e3435414667c44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scanparams-request)))
  "Returns md5sum for a message object of type 'scanparams-request"
  "0852ff892d6a7a08b5e3435414667c44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scanparams-request>)))
  "Returns full string definition for message of type '<scanparams-request>"
  (cl:format cl:nil "float64 lineangle~%float64 frameangle~%float64 maxlineangle~%float64 minlineangle~%float64 maxframeangle~%float64 minframeangle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scanparams-request)))
  "Returns full string definition for message of type 'scanparams-request"
  (cl:format cl:nil "float64 lineangle~%float64 frameangle~%float64 maxlineangle~%float64 minlineangle~%float64 maxframeangle~%float64 minframeangle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scanparams-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scanparams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'scanparams-request
    (cl:cons ':lineangle (lineangle msg))
    (cl:cons ':frameangle (frameangle msg))
    (cl:cons ':maxlineangle (maxlineangle msg))
    (cl:cons ':minlineangle (minlineangle msg))
    (cl:cons ':maxframeangle (maxframeangle msg))
    (cl:cons ':minframeangle (minframeangle msg))
))
;//! \htmlinclude scanparams-response.msg.html

(cl:defclass <scanparams-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass scanparams-response (<scanparams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scanparams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scanparams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name riegl-srv:<scanparams-response> is deprecated: use riegl-srv:scanparams-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scanparams-response>) ostream)
  "Serializes a message object of type '<scanparams-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scanparams-response>) istream)
  "Deserializes a message object of type '<scanparams-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scanparams-response>)))
  "Returns string type for a service object of type '<scanparams-response>"
  "riegl/scanparamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanparams-response)))
  "Returns string type for a service object of type 'scanparams-response"
  "riegl/scanparamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scanparams-response>)))
  "Returns md5sum for a message object of type '<scanparams-response>"
  "0852ff892d6a7a08b5e3435414667c44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scanparams-response)))
  "Returns md5sum for a message object of type 'scanparams-response"
  "0852ff892d6a7a08b5e3435414667c44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scanparams-response>)))
  "Returns full string definition for message of type '<scanparams-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scanparams-response)))
  "Returns full string definition for message of type 'scanparams-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scanparams-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scanparams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'scanparams-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'scanparams)))
  'scanparams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'scanparams)))
  'scanparams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanparams)))
  "Returns string type for a service object of type '<scanparams>"
  "riegl/scanparams")