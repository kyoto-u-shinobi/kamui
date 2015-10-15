
(cl:in-package :asdf)

(defsystem "can_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CANFrame" :depends-on ("_package_CANFrame"))
    (:file "_package_CANFrame" :depends-on ("_package"))
  ))