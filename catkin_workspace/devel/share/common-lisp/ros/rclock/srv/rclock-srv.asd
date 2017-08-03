
(cl:in-package :asdf)

(defsystem "rclock-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "logDir" :depends-on ("_package_logDir"))
    (:file "_package_logDir" :depends-on ("_package"))
  ))