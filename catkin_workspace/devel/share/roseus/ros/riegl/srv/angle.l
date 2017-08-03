;; Auto-generated. Do not edit!


(when (boundp 'riegl::angle)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'angle (find-package "RIEGL")))
(unless (find-package "RIEGL::ANGLE")
  (make-package "RIEGL::ANGLE"))
(unless (find-package "RIEGL::ANGLEREQUEST")
  (make-package "RIEGL::ANGLEREQUEST"))
(unless (find-package "RIEGL::ANGLERESPONSE")
  (make-package "RIEGL::ANGLERESPONSE"))

(in-package "ROS")





(defclass riegl::angleRequest
  :super ros::object
  :slots (_angle ))

(defmethod riegl::angleRequest
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass riegl::angleResponse
  :super ros::object
  :slots ())

(defmethod riegl::angleResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass riegl::angle
  :super ros::object
  :slots ())

(setf (get riegl::angle :md5sum-) "4edb55038e2b888976a0c0c56935341c")
(setf (get riegl::angle :datatype-) "riegl/angle")
(setf (get riegl::angle :request) riegl::angleRequest)
(setf (get riegl::angle :response) riegl::angleResponse)

(defmethod riegl::angleRequest
  (:response () (instance riegl::angleResponse :init)))

(setf (get riegl::angleRequest :md5sum-) "4edb55038e2b888976a0c0c56935341c")
(setf (get riegl::angleRequest :datatype-) "riegl/angleRequest")
(setf (get riegl::angleRequest :definition-)
      "float64 angle
---

")

(setf (get riegl::angleResponse :md5sum-) "4edb55038e2b888976a0c0c56935341c")
(setf (get riegl::angleResponse :datatype-) "riegl/angleResponse")
(setf (get riegl::angleResponse :definition-)
      "float64 angle
---

")



(provide :riegl/angle "4edb55038e2b888976a0c0c56935341c")


