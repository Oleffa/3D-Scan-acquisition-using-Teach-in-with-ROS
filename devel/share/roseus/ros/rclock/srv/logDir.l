;; Auto-generated. Do not edit!


(when (boundp 'rclock::logDir)
  (if (not (find-package "RCLOCK"))
    (make-package "RCLOCK"))
  (shadow 'logDir (find-package "RCLOCK")))
(unless (find-package "RCLOCK::LOGDIR")
  (make-package "RCLOCK::LOGDIR"))
(unless (find-package "RCLOCK::LOGDIRREQUEST")
  (make-package "RCLOCK::LOGDIRREQUEST"))
(unless (find-package "RCLOCK::LOGDIRRESPONSE")
  (make-package "RCLOCK::LOGDIRRESPONSE"))

(in-package "ROS")





(defclass rclock::logDirRequest
  :super ros::object
  :slots ())

(defmethod rclock::logDirRequest
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

(defclass rclock::logDirResponse
  :super ros::object
  :slots (_directory ))

(defmethod rclock::logDirResponse
  (:init
   (&key
    ((:directory __directory) "")
    )
   (send-super :init)
   (setq _directory (string __directory))
   self)
  (:directory
   (&optional __directory)
   (if __directory (setq _directory __directory)) _directory)
  (:serialization-length
   ()
   (+
    ;; string _directory
    4 (length _directory)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _directory
       (write-long (length _directory) s) (princ _directory s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _directory
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _directory (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rclock::logDir
  :super ros::object
  :slots ())

(setf (get rclock::logDir :md5sum-) "310e138069b8368226f05a8c4e7bb107")
(setf (get rclock::logDir :datatype-) "rclock/logDir")
(setf (get rclock::logDir :request) rclock::logDirRequest)
(setf (get rclock::logDir :response) rclock::logDirResponse)

(defmethod rclock::logDirRequest
  (:response () (instance rclock::logDirResponse :init)))

(setf (get rclock::logDirRequest :md5sum-) "310e138069b8368226f05a8c4e7bb107")
(setf (get rclock::logDirRequest :datatype-) "rclock/logDirRequest")
(setf (get rclock::logDirRequest :definition-)
      "---
string directory

")

(setf (get rclock::logDirResponse :md5sum-) "310e138069b8368226f05a8c4e7bb107")
(setf (get rclock::logDirResponse :datatype-) "rclock/logDirResponse")
(setf (get rclock::logDirResponse :definition-)
      "---
string directory

")



(provide :rclock/logDir "310e138069b8368226f05a8c4e7bb107")


