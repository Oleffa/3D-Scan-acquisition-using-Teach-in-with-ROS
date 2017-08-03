;; Auto-generated. Do not edit!


(when (boundp 'riegl::RieglStatus)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'RieglStatus (find-package "RIEGL")))
(unless (find-package "RIEGL::RIEGLSTATUS")
  (make-package "RIEGL::RIEGLSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RieglStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass riegl::RieglStatus
  :super ros::object
  :slots (_header _status ))

(defmethod riegl::RieglStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _status (round __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _status
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _status
       (write-long _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get riegl::RieglStatus :md5sum-) "464017b6d3bfd385079f3c7dc3b2e6be")
(setf (get riegl::RieglStatus :datatype-) "riegl/RieglStatus")
(setf (get riegl::RieglStatus :definition-)
      "Header header
# 0 scanning
# 1 frame start
# 2 frame stop
int32 status

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :riegl/RieglStatus "464017b6d3bfd385079f3c7dc3b2e6be")


