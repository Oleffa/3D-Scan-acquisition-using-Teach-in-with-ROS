
(cl:in-package :asdf)

(defsystem "volksbot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "vels" :depends-on ("_package_vels"))
    (:file "_package_vels" :depends-on ("_package"))
    (:file "pose2d" :depends-on ("_package_pose2d"))
    (:file "_package_pose2d" :depends-on ("_package"))
    (:file "ticks" :depends-on ("_package_ticks"))
    (:file "_package_ticks" :depends-on ("_package"))
  ))