
(cl:in-package :asdf)

(defsystem "viconros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "viconmocap" :depends-on ("_package_viconmocap"))
    (:file "_package_viconmocap" :depends-on ("_package"))
  ))