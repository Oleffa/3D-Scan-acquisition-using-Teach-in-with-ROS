;; Auto-generated. Do not edit!


(when (boundp 'bachelorarbeit::AddTwoInts)
  (if (not (find-package "BACHELORARBEIT"))
    (make-package "BACHELORARBEIT"))
  (shadow 'AddTwoInts (find-package "BACHELORARBEIT")))
(unless (find-package "BACHELORARBEIT::ADDTWOINTS")
  (make-package "BACHELORARBEIT::ADDTWOINTS"))
(unless (find-package "BACHELORARBEIT::ADDTWOINTSREQUEST")
  (make-package "BACHELORARBEIT::ADDTWOINTSREQUEST"))
(unless (find-package "BACHELORARBEIT::ADDTWOINTSRESPONSE")
  (make-package "BACHELORARBEIT::ADDTWOINTSRESPONSE"))

(in-package "ROS")





(defclass bachelorarbeit::AddTwoIntsRequest
  :super ros::object
  :slots (_a ))

(defmethod bachelorarbeit::AddTwoIntsRequest
  (:init
   (&key
    ((:a __a) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; int64 _a
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _a
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _a (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _a) (= (length (_a . bv)) 2)) ;; bignum
              (write-long (ash (elt (_a . bv) 0) 0) s)
              (write-long (ash (elt (_a . bv) 1) -1) s))
             ((and (class _a) (= (length (_a . bv)) 1)) ;; big1
              (write-long (elt (_a . bv) 0) s)
              (write-long (if (>= _a 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _a s)(write-long (if (>= _a 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _a
#+(or :alpha :irix6 :x86_64)
      (setf _a (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _a (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(defclass bachelorarbeit::AddTwoIntsResponse
  :super ros::object
  :slots (_sum ))

(defmethod bachelorarbeit::AddTwoIntsResponse
  (:init
   (&key
    ((:sum __sum) 0)
    )
   (send-super :init)
   (setq _sum (round __sum))
   self)
  (:sum
   (&optional __sum)
   (if __sum (setq _sum __sum)) _sum)
  (:serialization-length
   ()
   (+
    ;; int64 _sum
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _sum
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _sum (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _sum) (= (length (_sum . bv)) 2)) ;; bignum
              (write-long (ash (elt (_sum . bv) 0) 0) s)
              (write-long (ash (elt (_sum . bv) 1) -1) s))
             ((and (class _sum) (= (length (_sum . bv)) 1)) ;; big1
              (write-long (elt (_sum . bv) 0) s)
              (write-long (if (>= _sum 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _sum s)(write-long (if (>= _sum 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _sum
#+(or :alpha :irix6 :x86_64)
      (setf _sum (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _sum (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(defclass bachelorarbeit::AddTwoInts
  :super ros::object
  :slots ())

(setf (get bachelorarbeit::AddTwoInts :md5sum-) "0cd2157d7eaac1ba7a7e9dde0a510aef")
(setf (get bachelorarbeit::AddTwoInts :datatype-) "bachelorarbeit/AddTwoInts")
(setf (get bachelorarbeit::AddTwoInts :request) bachelorarbeit::AddTwoIntsRequest)
(setf (get bachelorarbeit::AddTwoInts :response) bachelorarbeit::AddTwoIntsResponse)

(defmethod bachelorarbeit::AddTwoIntsRequest
  (:response () (instance bachelorarbeit::AddTwoIntsResponse :init)))

(setf (get bachelorarbeit::AddTwoIntsRequest :md5sum-) "0cd2157d7eaac1ba7a7e9dde0a510aef")
(setf (get bachelorarbeit::AddTwoIntsRequest :datatype-) "bachelorarbeit/AddTwoIntsRequest")
(setf (get bachelorarbeit::AddTwoIntsRequest :definition-)
      "int64 a
---
int64 sum

")

(setf (get bachelorarbeit::AddTwoIntsResponse :md5sum-) "0cd2157d7eaac1ba7a7e9dde0a510aef")
(setf (get bachelorarbeit::AddTwoIntsResponse :datatype-) "bachelorarbeit/AddTwoIntsResponse")
(setf (get bachelorarbeit::AddTwoIntsResponse :definition-)
      "int64 a
---
int64 sum

")



(provide :bachelorarbeit/AddTwoInts "0cd2157d7eaac1ba7a7e9dde0a510aef")


