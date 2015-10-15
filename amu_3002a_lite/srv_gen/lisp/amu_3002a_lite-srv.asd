
(cl:in-package :asdf)

(defsystem "amu_3002a_lite-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "amu_control" :depends-on ("_package_amu_control"))
    (:file "_package_amu_control" :depends-on ("_package"))
  ))