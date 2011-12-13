
(cl:in-package :asdf)

(defsystem "hubo_ctrl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HuboCtrl" :depends-on ("_package_HuboCtrl"))
    (:file "_package_HuboCtrl" :depends-on ("_package"))
  ))