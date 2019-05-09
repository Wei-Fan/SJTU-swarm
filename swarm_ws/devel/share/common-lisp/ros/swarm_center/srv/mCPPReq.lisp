; Auto-generated. Do not edit!


(cl:in-package swarm_center-srv)


;//! \htmlinclude mCPPReq-request.msg.html

(cl:defclass <mCPPReq-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (y
    :reader y
    :initarg :y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass mCPPReq-request (<mCPPReq-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mCPPReq-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mCPPReq-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_center-srv:<mCPPReq-request> is deprecated: use swarm_center-srv:mCPPReq-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <mCPPReq-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_center-srv:x-val is deprecated.  Use swarm_center-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <mCPPReq-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_center-srv:y-val is deprecated.  Use swarm_center-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mCPPReq-request>) ostream)
  "Serializes a message object of type '<mCPPReq-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'y))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mCPPReq-request>) istream)
  "Deserializes a message object of type '<mCPPReq-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mCPPReq-request>)))
  "Returns string type for a service object of type '<mCPPReq-request>"
  "swarm_center/mCPPReqRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mCPPReq-request)))
  "Returns string type for a service object of type 'mCPPReq-request"
  "swarm_center/mCPPReqRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mCPPReq-request>)))
  "Returns md5sum for a message object of type '<mCPPReq-request>"
  "7b6a8c3e6d19ea93d36f2733e920800f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mCPPReq-request)))
  "Returns md5sum for a message object of type 'mCPPReq-request"
  "7b6a8c3e6d19ea93d36f2733e920800f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mCPPReq-request>)))
  "Returns full string definition for message of type '<mCPPReq-request>"
  (cl:format cl:nil "float32[] x~%float32[] y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mCPPReq-request)))
  "Returns full string definition for message of type 'mCPPReq-request"
  (cl:format cl:nil "float32[] x~%float32[] y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mCPPReq-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mCPPReq-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mCPPReq-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
;//! \htmlinclude mCPPReq-response.msg.html

(cl:defclass <mCPPReq-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mCPPReq-response (<mCPPReq-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mCPPReq-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mCPPReq-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_center-srv:<mCPPReq-response> is deprecated: use swarm_center-srv:mCPPReq-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <mCPPReq-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_center-srv:b-val is deprecated.  Use swarm_center-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mCPPReq-response>) ostream)
  "Serializes a message object of type '<mCPPReq-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mCPPReq-response>) istream)
  "Deserializes a message object of type '<mCPPReq-response>"
    (cl:setf (cl:slot-value msg 'b) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mCPPReq-response>)))
  "Returns string type for a service object of type '<mCPPReq-response>"
  "swarm_center/mCPPReqResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mCPPReq-response)))
  "Returns string type for a service object of type 'mCPPReq-response"
  "swarm_center/mCPPReqResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mCPPReq-response>)))
  "Returns md5sum for a message object of type '<mCPPReq-response>"
  "7b6a8c3e6d19ea93d36f2733e920800f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mCPPReq-response)))
  "Returns md5sum for a message object of type 'mCPPReq-response"
  "7b6a8c3e6d19ea93d36f2733e920800f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mCPPReq-response>)))
  "Returns full string definition for message of type '<mCPPReq-response>"
  (cl:format cl:nil "bool b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mCPPReq-response)))
  "Returns full string definition for message of type 'mCPPReq-response"
  (cl:format cl:nil "bool b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mCPPReq-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mCPPReq-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mCPPReq-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mCPPReq)))
  'mCPPReq-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mCPPReq)))
  'mCPPReq-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mCPPReq)))
  "Returns string type for a service object of type '<mCPPReq>"
  "swarm_center/mCPPReq")