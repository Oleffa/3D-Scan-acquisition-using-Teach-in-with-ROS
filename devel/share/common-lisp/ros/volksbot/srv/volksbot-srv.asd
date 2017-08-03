
(cl:in-package :asdf)

(defsystem "volksbot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "velocities" :depends-on ("_package_velocities"))
    (:file "_package_velocities" :depends-on ("_package"))
  ))