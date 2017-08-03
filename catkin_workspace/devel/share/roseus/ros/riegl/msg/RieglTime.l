;; Auto-generated. Do not edit!


(when (boundp 'riegl::RieglTime)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'RieglTime (find-package "RIEGL")))
(unless (find-package "RIEGL::RIEGLTIME")
  (make-package "RIEGL::RIEGLTIME"))

(in-package "ROS")
;;//! \htmlinclude RieglTime.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass riegl::RieglTime
  :super ros::object
  :slots (_header _time ))

(defmethod riegl::RieglTime
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time (float __time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get riegl::RieglTime :md5sum-) "dce2d1e2213c6c29642ee1125033b12b")
(setf (get riegl::RieglTime :datatype-) "riegl/RieglTime")
(setf (get riegl::RieglTime :definition-)
      "Header header
# 0 scanning
# 1 frame start
# 2 frame stop
float64 time

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



(provide :riegl/RieglTime "dce2d1e2213c6c29642ee1125033b12b")


