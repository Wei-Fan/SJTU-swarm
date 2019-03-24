;; Auto-generated. Do not edit!


(when (boundp 'swarm_center::mCPPReq)
  (if (not (find-package "SWARM_CENTER"))
    (make-package "SWARM_CENTER"))
  (shadow 'mCPPReq (find-package "SWARM_CENTER")))
(unless (find-package "SWARM_CENTER::MCPPREQ")
  (make-package "SWARM_CENTER::MCPPREQ"))
(unless (find-package "SWARM_CENTER::MCPPREQREQUEST")
  (make-package "SWARM_CENTER::MCPPREQREQUEST"))
(unless (find-package "SWARM_CENTER::MCPPREQRESPONSE")
  (make-package "SWARM_CENTER::MCPPREQRESPONSE"))

(in-package "ROS")





(defclass swarm_center::mCPPReqRequest
  :super ros::object
  :slots (_a ))

(defmethod swarm_center::mCPPReqRequest
  (:init
   (&key
    ((:a __a) nil)
    )
   (send-super :init)
   (setq _a __a)
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; bool _a
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _a
       (if _a (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _a
     (setq _a (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass swarm_center::mCPPReqResponse
  :super ros::object
  :slots (_b ))

(defmethod swarm_center::mCPPReqResponse
  (:init
   (&key
    ((:b __b) nil)
    )
   (send-super :init)
   (setq _b __b)
   self)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; bool _b
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _b
       (if _b (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _b
     (setq _b (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass swarm_center::mCPPReq
  :super ros::object
  :slots ())

(setf (get swarm_center::mCPPReq :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get swarm_center::mCPPReq :datatype-) "swarm_center/mCPPReq")
(setf (get swarm_center::mCPPReq :request) swarm_center::mCPPReqRequest)
(setf (get swarm_center::mCPPReq :response) swarm_center::mCPPReqResponse)

(defmethod swarm_center::mCPPReqRequest
  (:response () (instance swarm_center::mCPPReqResponse :init)))

(setf (get swarm_center::mCPPReqRequest :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get swarm_center::mCPPReqRequest :datatype-) "swarm_center/mCPPReqRequest")
(setf (get swarm_center::mCPPReqRequest :definition-)
      "bool a
---
bool b

")

(setf (get swarm_center::mCPPReqResponse :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get swarm_center::mCPPReqResponse :datatype-) "swarm_center/mCPPReqResponse")
(setf (get swarm_center::mCPPReqResponse :definition-)
      "bool a
---
bool b

")



(provide :swarm_center/mCPPReq "81f01bfd9a951b1adf9102125874ff5b")


