
(cl:in-package :asdf)

(defsystem "swarm_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "pos_info" :depends-on ("_package_pos_info"))
    (:file "_package_pos_info" :depends-on ("_package"))
  ))