; Auto-generated. Do not edit!


(cl:in-package rclock-srv)


;//! \htmlinclude logDir-request.msg.html

(cl:defclass <logDir-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass logDir-request (<logDir-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <logDir-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'logDir-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rclock-srv:<logDir-request> is deprecated: use rclock-srv:logDir-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <logDir-request>) ostream)
  "Serializes a message object of type '<logDir-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <logDir-request>) istream)
  "Deserializes a message object of type '<logDir-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<logDir-request>)))
  "Returns string type for a service object of type '<logDir-request>"
  "rclock/logDirRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'logDir-request)))
  "Returns string type for a service object of type 'logDir-request"
  "rclock/logDirRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<logDir-request>)))
  "Returns md5sum for a message object of type '<logDir-request>"
  "310e138069b8368226f05a8c4e7bb107")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'logDir-request)))
  "Returns md5sum for a message object of type 'logDir-request"
  "310e138069b8368226f05a8c4e7bb107")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<logDir-request>)))
  "Returns full string definition for message of type '<logDir-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'logDir-request)))
  "Returns full string definition for message of type 'logDir-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <logDir-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <logDir-request>))
  "Converts a ROS message object to a list"
  (cl:list 'logDir-request
))
;//! \htmlinclude logDir-response.msg.html

(cl:defclass <logDir-response> (roslisp-msg-protocol:ros-message)
  ((directory
    :reader directory
    :initarg :directory
    :type cl:string
    :initform ""))
)

(cl:defclass logDir-response (<logDir-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <logDir-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'logDir-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rclock-srv:<logDir-response> is deprecated: use rclock-srv:logDir-response instead.")))

(cl:ensure-generic-function 'directory-val :lambda-list '(m))
(cl:defmethod directory-val ((m <logDir-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rclock-srv:directory-val is deprecated.  Use rclock-srv:directory instead.")
  (directory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <logDir-response>) ostream)
  "Serializes a message object of type '<logDir-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'directory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'directory))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <logDir-response>) istream)
  "Deserializes a message object of type '<logDir-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'directory) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'directory) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<logDir-response>)))
  "Returns string type for a service object of type '<logDir-response>"
  "rclock/logDirResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'logDir-response)))
  "Returns string type for a service object of type 'logDir-response"
  "rclock/logDirResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<logDir-response>)))
  "Returns md5sum for a message object of type '<logDir-response>"
  "310e138069b8368226f05a8c4e7bb107")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'logDir-response)))
  "Returns md5sum for a message object of type 'logDir-response"
  "310e138069b8368226f05a8c4e7bb107")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<logDir-response>)))
  "Returns full string definition for message of type '<logDir-response>"
  (cl:format cl:nil "string directory~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'logDir-response)))
  "Returns full string definition for message of type 'logDir-response"
  (cl:format cl:nil "string directory~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <logDir-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'directory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <logDir-response>))
  "Converts a ROS message object to a list"
  (cl:list 'logDir-response
    (cl:cons ':directory (directory msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'logDir)))
  'logDir-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'logDir)))
  'logDir-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'logDir)))
  "Returns string type for a service object of type '<logDir>"
  "rclock/logDir")