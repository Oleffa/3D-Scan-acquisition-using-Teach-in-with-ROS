;; Auto-generated. Do not edit!


(when (boundp 'riegl::progress)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'progress (find-package "RIEGL")))
(unless (find-package "RIEGL::PROGRESS")
  (make-package "RIEGL::PROGRESS"))
(unless (find-package "RIEGL::PROGRESSREQUEST")
  (make-package "RIEGL::PROGRESSREQUEST"))
(unless (find-package "RIEGL::PROGRESSRESPONSE")
  (make-package "RIEGL::PROGRESSRESPONSE"))

(in-package "ROS")





(defclass riegl::progressRequest
  :super ros::object
  :slots ())

(defmethod riegl::progressRequest
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

(defclass riegl::progressResponse
  :super ros::object
  :slots (_progress ))

(defmethod riegl::progressResponse
  (:init
   (&key
    ((:progress __progress) 0.0)
    )
   (send-super :init)
   (setq _progress (float __progress))
   self)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
  (:serialization-length
   ()
   (+
    ;; float64 _progress
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _progress
       (sys::poke _progress (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _progress
     (setq _progress (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass riegl::progress
  :super ros::object
  :slots ())

(setf (get riegl::progress :md5sum-) "7afca0099e0cddc25243b1e3569895fe")
(setf (get riegl::progress :datatype-) "riegl/progress")
(setf (get riegl::progress :request) riegl::progressRequest)
(setf (get riegl::progress :response) riegl::progressResponse)

(defmethod riegl::progressRequest
  (:response () (instance riegl::progressResponse :init)))

(setf (get riegl::progressRequest :md5sum-) "7afca0099e0cddc25243b1e3569895fe")
(setf (get riegl::progressRequest :datatype-) "riegl/progressRequest")
(setf (get riegl::progressRequest :definition-)
      "---
float64 progress

")

(setf (get riegl::progressResponse :md5sum-) "7afca0099e0cddc25243b1e3569895fe")
(setf (get riegl::progressResponse :datatype-) "riegl/progressResponse")
(setf (get riegl::progressResponse :definition-)
      "---
float64 progress

")



(provide :riegl/progress "7afca0099e0cddc25243b1e3569895fe")


