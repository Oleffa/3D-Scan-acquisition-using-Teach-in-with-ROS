
(cl:in-package :asdf)

(defsystem "riegl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "scanparams" :depends-on ("_package_scanparams"))
    (:file "_package_scanparams" :depends-on ("_package"))
    (:file "angle" :depends-on ("_package_angle"))
    (:file "_package_angle" :depends-on ("_package"))
    (:file "selection" :depends-on ("_package_selection"))
    (:file "_package_selection" :depends-on ("_package"))
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "progress" :depends-on ("_package_progress"))
    (:file "_package_progress" :depends-on ("_package"))
    (:file "inclination" :depends-on ("_package_inclination"))
    (:file "_package_inclination" :depends-on ("_package"))
  ))