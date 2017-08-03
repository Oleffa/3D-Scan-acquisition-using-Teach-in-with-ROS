
(cl:in-package :asdf)

(defsystem "riegl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RieglTime" :depends-on ("_package_RieglTime"))
    (:file "_package_RieglTime" :depends-on ("_package"))
    (:file "RieglLine" :depends-on ("_package_RieglLine"))
    (:file "_package_RieglLine" :depends-on ("_package"))
    (:file "RieglStatus" :depends-on ("_package_RieglStatus"))
    (:file "_package_RieglStatus" :depends-on ("_package"))
  ))