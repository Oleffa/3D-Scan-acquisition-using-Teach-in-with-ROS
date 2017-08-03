;; Auto-generated. Do not edit!


(when (boundp 'volksbot::pose2d)
  (if (not (find-package "VOLKSBOT"))
    (make-package "VOLKSBOT"))
  (shadow 'pose2d (find-package "VOLKSBOT")))
(unless (find-package "VOLKSBOT::POSE2D")
  (make-package "VOLKSBOT::POSE2D"))

(in-package "ROS")
;;//! \htmlinclude pose2d.msg.html


(defclass volksbot::pose2d
  :super ros::object
  :slots (_x _z _theta ))

(defmethod volksbot::pose2d
  (:init
   (&key
    ((:x __x) 0.0)
    ((:z __z) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _z (float __z))
   (setq _theta (float __theta))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _z
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get volksbot::pose2d :md5sum-) "38f95d136be6e97064dca7adba40b51c")
(setf (get volksbot::pose2d :datatype-) "volksbot/pose2d")
(setf (get volksbot::pose2d :definition-)
      "float64 x
float64 z
float64 theta

")



(provide :volksbot/pose2d "38f95d136be6e97064dca7adba40b51c")


