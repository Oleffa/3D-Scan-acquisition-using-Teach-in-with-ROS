;; Auto-generated. Do not edit!


(when (boundp 'riegl::scanparams)
  (if (not (find-package "RIEGL"))
    (make-package "RIEGL"))
  (shadow 'scanparams (find-package "RIEGL")))
(unless (find-package "RIEGL::SCANPARAMS")
  (make-package "RIEGL::SCANPARAMS"))
(unless (find-package "RIEGL::SCANPARAMSREQUEST")
  (make-package "RIEGL::SCANPARAMSREQUEST"))
(unless (find-package "RIEGL::SCANPARAMSRESPONSE")
  (make-package "RIEGL::SCANPARAMSRESPONSE"))

(in-package "ROS")





(defclass riegl::scanparamsRequest
  :super ros::object
  :slots (_lineangle _frameangle _maxlineangle _minlineangle _maxframeangle _minframeangle ))

(defmethod riegl::scanparamsRequest
  (:init
   (&key
    ((:lineangle __lineangle) 0.0)
    ((:frameangle __frameangle) 0.0)
    ((:maxlineangle __maxlineangle) 0.0)
    ((:minlineangle __minlineangle) 0.0)
    ((:maxframeangle __maxframeangle) 0.0)
    ((:minframeangle __minframeangle) 0.0)
    )
   (send-super :init)
   (setq _lineangle (float __lineangle))
   (setq _frameangle (float __frameangle))
   (setq _maxlineangle (float __maxlineangle))
   (setq _minlineangle (float __minlineangle))
   (setq _maxframeangle (float __maxframeangle))
   (setq _minframeangle (float __minframeangle))
   self)
  (:lineangle
   (&optional __lineangle)
   (if __lineangle (setq _lineangle __lineangle)) _lineangle)
  (:frameangle
   (&optional __frameangle)
   (if __frameangle (setq _frameangle __frameangle)) _frameangle)
  (:maxlineangle
   (&optional __maxlineangle)
   (if __maxlineangle (setq _maxlineangle __maxlineangle)) _maxlineangle)
  (:minlineangle
   (&optional __minlineangle)
   (if __minlineangle (setq _minlineangle __minlineangle)) _minlineangle)
  (:maxframeangle
   (&optional __maxframeangle)
   (if __maxframeangle (setq _maxframeangle __maxframeangle)) _maxframeangle)
  (:minframeangle
   (&optional __minframeangle)
   (if __minframeangle (setq _minframeangle __minframeangle)) _minframeangle)
  (:serialization-length
   ()
   (+
    ;; float64 _lineangle
    8
    ;; float64 _frameangle
    8
    ;; float64 _maxlineangle
    8
    ;; float64 _minlineangle
    8
    ;; float64 _maxframeangle
    8
    ;; float64 _minframeangle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _lineangle
       (sys::poke _lineangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _frameangle
       (sys::poke _frameangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxlineangle
       (sys::poke _maxlineangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _minlineangle
       (sys::poke _minlineangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxframeangle
       (sys::poke _maxframeangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _minframeangle
       (sys::poke _minframeangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _lineangle
     (setq _lineangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _frameangle
     (setq _frameangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxlineangle
     (setq _maxlineangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _minlineangle
     (setq _minlineangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxframeangle
     (setq _maxframeangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _minframeangle
     (setq _minframeangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass riegl::scanparamsResponse
  :super ros::object
  :slots ())

(defmethod riegl::scanparamsResponse
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

(defclass riegl::scanparams
  :super ros::object
  :slots ())

(setf (get riegl::scanparams :md5sum-) "0852ff892d6a7a08b5e3435414667c44")
(setf (get riegl::scanparams :datatype-) "riegl/scanparams")
(setf (get riegl::scanparams :request) riegl::scanparamsRequest)
(setf (get riegl::scanparams :response) riegl::scanparamsResponse)

(defmethod riegl::scanparamsRequest
  (:response () (instance riegl::scanparamsResponse :init)))

(setf (get riegl::scanparamsRequest :md5sum-) "0852ff892d6a7a08b5e3435414667c44")
(setf (get riegl::scanparamsRequest :datatype-) "riegl/scanparamsRequest")
(setf (get riegl::scanparamsRequest :definition-)
      "float64 lineangle
float64 frameangle
float64 maxlineangle
float64 minlineangle
float64 maxframeangle
float64 minframeangle
---

")

(setf (get riegl::scanparamsResponse :md5sum-) "0852ff892d6a7a08b5e3435414667c44")
(setf (get riegl::scanparamsResponse :datatype-) "riegl/scanparamsResponse")
(setf (get riegl::scanparamsResponse :definition-)
      "float64 lineangle
float64 frameangle
float64 maxlineangle
float64 minlineangle
float64 maxframeangle
float64 minframeangle
---

")



(provide :riegl/scanparams "0852ff892d6a7a08b5e3435414667c44")


