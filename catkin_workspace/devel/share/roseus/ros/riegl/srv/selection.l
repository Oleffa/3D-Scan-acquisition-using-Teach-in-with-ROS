;; Auto-generated. Do not edit!


(when (boundp 'riegl::selection)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'selection (find-package "RIEGL")))
(unless (find-package "RIEGL::SELECTION")
  (make-package "RIEGL::SELECTION"))
(unless (find-package "RIEGL::SELECTIONREQUEST")
  (make-package "RIEGL::SELECTIONREQUEST"))
(unless (find-package "RIEGL::SELECTIONRESPONSE")
  (make-package "RIEGL::SELECTIONRESPONSE"))

(in-package "ROS")





(defclass riegl::selectionRequest
  :super ros::object
  :slots (_type ))

(defmethod riegl::selectionRequest
  (:init
   (&key
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; int64 _type
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _type) (= (length (_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_type . bv) 0) 0) s)
              (write-long (ash (elt (_type . bv) 1) -1) s))
             ((and (class _type) (= (length (_type . bv)) 1)) ;; big1
              (write-long (elt (_type . bv) 0) s)
              (write-long (if (>= _type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _type s)(write-long (if (>= _type 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _type
#+(or :alpha :irix6 :x86_64)
      (setf _type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass riegl::selectionResponse
  :super ros::object
  :slots ())

(defmethod riegl::selectionResponse
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

(defclass riegl::selection
  :super ros::object
  :slots ())

(setf (get riegl::selection :md5sum-) "79260b638c2b88f6366fba8cd5157389")
(setf (get riegl::selection :datatype-) "riegl/selection")
(setf (get riegl::selection :request) riegl::selectionRequest)
(setf (get riegl::selection :response) riegl::selectionResponse)

(defmethod riegl::selectionRequest
  (:response () (instance riegl::selectionResponse :init)))

(setf (get riegl::selectionRequest :md5sum-) "79260b638c2b88f6366fba8cd5157389")
(setf (get riegl::selectionRequest :datatype-) "riegl/selectionRequest")
(setf (get riegl::selectionRequest :definition-)
      "int64 type
---

")

(setf (get riegl::selectionResponse :md5sum-) "79260b638c2b88f6366fba8cd5157389")
(setf (get riegl::selectionResponse :datatype-) "riegl/selectionResponse")
(setf (get riegl::selectionResponse :definition-)
      "int64 type
---

")



(provide :riegl/selection "79260b638c2b88f6366fba8cd5157389")


