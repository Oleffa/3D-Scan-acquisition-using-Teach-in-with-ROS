;; Auto-generated. Do not edit!


(when (boundp 'riegl::Command)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'Command (find-package "RIEGL")))
(unless (find-package "RIEGL::COMMAND")
  (make-package "RIEGL::COMMAND"))
(unless (find-package "RIEGL::COMMANDREQUEST")
  (make-package "RIEGL::COMMANDREQUEST"))
(unless (find-package "RIEGL::COMMANDRESPONSE")
  (make-package "RIEGL::COMMANDRESPONSE"))

(in-package "ROS")





(defclass riegl::CommandRequest
  :super ros::object
  :slots (_command _params ))

(defmethod riegl::CommandRequest
  (:init
   (&key
    ((:command __command) "")
    ((:params __params) "")
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _params (string __params))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:params
   (&optional __params)
   (if __params (setq _params __params)) _params)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _params
    4 (length _params)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _params
       (write-long (length _params) s) (princ _params s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _params
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _params (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass riegl::CommandResponse
  :super ros::object
  :slots ())

(defmethod riegl::CommandResponse
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

(defclass riegl::Command
  :super ros::object
  :slots ())

(setf (get riegl::Command :md5sum-) "c8a9e93ba36acade4a5f63ab5669b103")
(setf (get riegl::Command :datatype-) "riegl/Command")
(setf (get riegl::Command :request) riegl::CommandRequest)
(setf (get riegl::Command :response) riegl::CommandResponse)

(defmethod riegl::CommandRequest
  (:response () (instance riegl::CommandResponse :init)))

(setf (get riegl::CommandRequest :md5sum-) "c8a9e93ba36acade4a5f63ab5669b103")
(setf (get riegl::CommandRequest :datatype-) "riegl/CommandRequest")
(setf (get riegl::CommandRequest :definition-)
      "string command
string params
---

")

(setf (get riegl::CommandResponse :md5sum-) "c8a9e93ba36acade4a5f63ab5669b103")
(setf (get riegl::CommandResponse :datatype-) "riegl/CommandResponse")
(setf (get riegl::CommandResponse :definition-)
      "string command
string params
---

")



(provide :riegl/Command "c8a9e93ba36acade4a5f63ab5669b103")


