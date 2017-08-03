;; Auto-generated. Do not edit!


(when (boundp 'riegl::inclination)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'inclination (find-package "RIEGL")))
(unless (find-package "RIEGL::INCLINATION")
  (make-package "RIEGL::INCLINATION"))
(unless (find-package "RIEGL::INCLINATIONREQUEST")
  (make-package "RIEGL::INCLINATIONREQUEST"))
(unless (find-package "RIEGL::INCLINATIONRESPONSE")
  (make-package "RIEGL::INCLINATIONRESPONSE"))

(in-package "ROS")





(defclass riegl::inclinationRequest
  :super ros::object
  :slots ())

(defmethod riegl::inclinationRequest
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

(defclass riegl::inclinationResponse
  :super ros::object
  :slots (_roll _pitch ))

(defmethod riegl::inclinationResponse
  (:init
   (&key
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    )
   (send-super :init)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   self)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:serialization-length
   ()
   (+
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass riegl::inclination
  :super ros::object
  :slots ())

(setf (get riegl::inclination :md5sum-) "0f5aa311af37faead3f9e90f420c9603")
(setf (get riegl::inclination :datatype-) "riegl/inclination")
(setf (get riegl::inclination :request) riegl::inclinationRequest)
(setf (get riegl::inclination :response) riegl::inclinationResponse)

(defmethod riegl::inclinationRequest
  (:response () (instance riegl::inclinationResponse :init)))

(setf (get riegl::inclinationRequest :md5sum-) "0f5aa311af37faead3f9e90f420c9603")
(setf (get riegl::inclinationRequest :datatype-) "riegl/inclinationRequest")
(setf (get riegl::inclinationRequest :definition-)
      "---
float64 roll
float64 pitch

")

(setf (get riegl::inclinationResponse :md5sum-) "0f5aa311af37faead3f9e90f420c9603")
(setf (get riegl::inclinationResponse :datatype-) "riegl/inclinationResponse")
(setf (get riegl::inclinationResponse :definition-)
      "---
float64 roll
float64 pitch

")



(provide :riegl/inclination "0f5aa311af37faead3f9e90f420c9603")


