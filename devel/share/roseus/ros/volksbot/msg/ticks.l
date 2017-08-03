;; Auto-generated. Do not edit!


(when (boundp 'volksbot::ticks)
  (if (not (find-package "VOLKSBOT"))
    (make-package "VOLKSBOT"))
  (shadow 'ticks (find-package "VOLKSBOT")))
(unless (find-package "VOLKSBOT::TICKS")
  (make-package "VOLKSBOT::TICKS"))

(in-package "ROS")
;;//! \htmlinclude ticks.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass volksbot::ticks
  :super ros::object
  :slots (_header _left _right _vx _vth ))

(defmethod volksbot::ticks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left __left) 0)
    ((:right __right) 0)
    ((:vx __vx) 0.0)
    ((:vth __vth) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left (round __left))
   (setq _right (round __right))
   (setq _vx (float __vx))
   (setq _vth (float __vth))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vth
   (&optional __vth)
   (if __vth (setq _vth __vth)) _vth)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _left
    4
    ;; int32 _right
    4
    ;; float32 _vx
    4
    ;; float32 _vth
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _left
       (write-long _left s)
     ;; int32 _right
       (write-long _right s)
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vth
       (sys::poke _vth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _left
     (setq _left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right
     (setq _right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vth
     (setq _vth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get volksbot::ticks :md5sum-) "2672a539e3db4fc52f0208a4f4c3936a")
(setf (get volksbot::ticks :datatype-) "volksbot/ticks")
(setf (get volksbot::ticks :definition-)
      "Header header
int32 left
int32 right
float32 vx
float32 vth

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



(provide :volksbot/ticks "2672a539e3db4fc52f0208a4f4c3936a")


