
(cl:in-package :asdf)

(defsystem "swarm_center-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "mArmReq" :depends-on ("_package_mArmReq"))
    (:file "_package_mArmReq" :depends-on ("_package"))
    (:file "mCPPReq" :depends-on ("_package_mCPPReq"))
    (:file "_package_mCPPReq" :depends-on ("_package"))
  ))