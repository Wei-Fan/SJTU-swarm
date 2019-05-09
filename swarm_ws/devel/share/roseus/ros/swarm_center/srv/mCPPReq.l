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
  :slots (_x _y ))

(defmethod swarm_center::mCPPReqRequest
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32[] _x
    (* 4    (length _x)) 4
    ;; float32[] _y
    (* 4    (length _y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
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

(setf (get swarm_center::mCPPReq :md5sum-) "7b6a8c3e6d19ea93d36f2733e920800f")
(setf (get swarm_center::mCPPReq :datatype-) "swarm_center/mCPPReq")
(setf (get swarm_center::mCPPReq :request) swarm_center::mCPPReqRequest)
(setf (get swarm_center::mCPPReq :response) swarm_center::mCPPReqResponse)

(defmethod swarm_center::mCPPReqRequest
  (:response () (instance swarm_center::mCPPReqResponse :init)))

(setf (get swarm_center::mCPPReqRequest :md5sum-) "7b6a8c3e6d19ea93d36f2733e920800f")
(setf (get swarm_center::mCPPReqRequest :datatype-) "swarm_center/mCPPReqRequest")
(setf (get swarm_center::mCPPReqRequest :definition-)
      "float32[] x
float32[] y
---
bool b

")

(setf (get swarm_center::mCPPReqResponse :md5sum-) "7b6a8c3e6d19ea93d36f2733e920800f")
(setf (get swarm_center::mCPPReqResponse :datatype-) "swarm_center/mCPPReqResponse")
(setf (get swarm_center::mCPPReqResponse :definition-)
      "float32[] x
float32[] y
---
bool b

")



(provide :swarm_center/mCPPReq "7b6a8c3e6d19ea93d36f2733e920800f")


