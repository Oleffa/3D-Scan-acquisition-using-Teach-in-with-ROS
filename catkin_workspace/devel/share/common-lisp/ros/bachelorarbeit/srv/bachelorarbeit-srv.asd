
(cl:in-package :asdf)

(defsystem "bachelorarbeit-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pressed" :depends-on ("_package_pressed"))
    (:file "_package_pressed" :depends-on ("_package"))
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
  ))