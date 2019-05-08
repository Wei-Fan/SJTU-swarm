;; Auto-generated. Do not edit!


(when (boundp 'swarm_center::mArmReq)
  (if (not (find-package "SWARM_CENTER"))
    (make-package "SWARM_CENTER"))
  (shadow 'mArmReq (find-package "SWARM_CENTER")))
(unless (find-package "SWARM_CENTER::MARMREQ")
  (make-package "SWARM_CENTER::MARMREQ"))
(unless (find-package "SWARM_CENTER::MARMREQREQUEST")
  (make-package "SWARM_CENTER::MARMREQREQUEST"))
(unless (find-package "SWARM_CENTER::MARMREQRESPONSE")
  (make-package "SWARM_CENTER::MARMREQRESPONSE"))

(in-package "ROS")





(defclass swarm_center::mArmReqRequest
  :super ros::object
  :slots (_a ))

(defmethod swarm_center::mArmReqRequest
  (:init
   (&key
    ((:a __a) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; int32 _a
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _a
       (write-long _a s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _a
     (setq _a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass swarm_center::mArmReqResponse
  :super ros::object
  :slots (_b ))

(defmethod swarm_center::mArmReqResponse
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

(defclass swarm_center::mArmReq
  :super ros::object
  :slots ())

(setf (get swarm_center::mArmReq :md5sum-) "cf33071162f00c3ef34465803c52a01d")
(setf (get swarm_center::mArmReq :datatype-) "swarm_center/mArmReq")
(setf (get swarm_center::mArmReq :request) swarm_center::mArmReqRequest)
(setf (get swarm_center::mArmReq :response) swarm_center::mArmReqResponse)

(defmethod swarm_center::mArmReqRequest
  (:response () (instance swarm_center::mArmReqResponse :init)))

(setf (get swarm_center::mArmReqRequest :md5sum-) "cf33071162f00c3ef34465803c52a01d")
(setf (get swarm_center::mArmReqRequest :datatype-) "swarm_center/mArmReqRequest")
(setf (get swarm_center::mArmReqRequest :definition-)
      "int32 a
---
bool b

")

(setf (get swarm_center::mArmReqResponse :md5sum-) "cf33071162f00c3ef34465803c52a01d")
(setf (get swarm_center::mArmReqResponse :datatype-) "swarm_center/mArmReqResponse")
(setf (get swarm_center::mArmReqResponse :definition-)
      "int32 a
---
bool b

")



(provide :swarm_center/mArmReq "cf33071162f00c3ef34465803c52a01d")


