;; Auto-generated. Do not edit!


(when (boundp 'volksbot::velocities)
  (if (not (find-package "VOLKSBOT"))
    (make-package "VOLKSBOT"))
  (shadow 'velocities (find-package "VOLKSBOT")))
(unless (find-package "VOLKSBOT::VELOCITIES")
  (make-package "VOLKSBOT::VELOCITIES"))
(unless (find-package "VOLKSBOT::VELOCITIESREQUEST")
  (make-package "VOLKSBOT::VELOCITIESREQUEST"))
(unless (find-package "VOLKSBOT::VELOCITIESRESPONSE")
  (make-package "VOLKSBOT::VELOCITIESRESPONSE"))

(in-package "ROS")





(defclass volksbot::velocitiesRequest
  :super ros::object
  :slots (_left _right ))

(defmethod volksbot::velocitiesRequest
  (:init
   (&key
    ((:left __left) 0.0)
    ((:right __right) 0.0)
    )
   (send-super :init)
   (setq _left (float __left))
   (setq _right (float __right))
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; float64 _left
    8
    ;; float64 _right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _left
       (sys::poke _left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right
       (sys::poke _right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _left
     (setq _left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right
     (setq _right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass volksbot::velocitiesResponse
  :super ros::object
  :slots ())

(defmethod volksbot::velocitiesResponse
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

(defclass volksbot::velocities
  :super ros::object
  :slots ())

(setf (get volksbot::velocities :md5sum-) "50c2436c38cded221d061b57126c4e40")
(setf (get volksbot::velocities :datatype-) "volksbot/velocities")
(setf (get volksbot::velocities :request) volksbot::velocitiesRequest)
(setf (get volksbot::velocities :response) volksbot::velocitiesResponse)

(defmethod volksbot::velocitiesRequest
  (:response () (instance volksbot::velocitiesResponse :init)))

(setf (get volksbot::velocitiesRequest :md5sum-) "50c2436c38cded221d061b57126c4e40")
(setf (get volksbot::velocitiesRequest :datatype-) "volksbot/velocitiesRequest")
(setf (get volksbot::velocitiesRequest :definition-)
      "float64 left
float64 right
---

")

(setf (get volksbot::velocitiesResponse :md5sum-) "50c2436c38cded221d061b57126c4e40")
(setf (get volksbot::velocitiesResponse :datatype-) "volksbot/velocitiesResponse")
(setf (get volksbot::velocitiesResponse :definition-)
      "float64 left
float64 right
---

")



(provide :volksbot/velocities "50c2436c38cded221d061b57126c4e40")


