; Auto-generated. Do not edit!


(cl:in-package swarm_center-srv)


;//! \htmlinclude mArmReq-request.msg.html

(cl:defclass <mArmReq-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0))
)

(cl:defclass mArmReq-request (<mArmReq-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mArmReq-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mArmReq-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_center-srv:<mArmReq-request> is deprecated: use swarm_center-srv:mArmReq-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <mArmReq-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_center-srv:a-val is deprecated.  Use swarm_center-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mArmReq-request>) ostream)
  "Serializes a message object of type '<mArmReq-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mArmReq-request>) istream)
  "Deserializes a message object of type '<mArmReq-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mArmReq-request>)))
  "Returns string type for a service object of type '<mArmReq-request>"
  "swarm_center/mArmReqRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mArmReq-request)))
  "Returns string type for a service object of type 'mArmReq-request"
  "swarm_center/mArmReqRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mArmReq-request>)))
  "Returns md5sum for a message object of type '<mArmReq-request>"
  "cf33071162f00c3ef34465803c52a01d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mArmReq-request)))
  "Returns md5sum for a message object of type 'mArmReq-request"
  "cf33071162f00c3ef34465803c52a01d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mArmReq-request>)))
  "Returns full string definition for message of type '<mArmReq-request>"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mArmReq-request)))
  "Returns full string definition for message of type 'mArmReq-request"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mArmReq-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mArmReq-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mArmReq-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude mArmReq-response.msg.html

(cl:defclass <mArmReq-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mArmReq-response (<mArmReq-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mArmReq-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mArmReq-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_center-srv:<mArmReq-response> is deprecated: use swarm_center-srv:mArmReq-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <mArmReq-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_center-srv:b-val is deprecated.  Use swarm_center-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mArmReq-response>) ostream)
  "Serializes a message object of type '<mArmReq-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mArmReq-response>) istream)
  "Deserializes a message object of type '<mArmReq-response>"
    (cl:setf (cl:slot-value msg 'b) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mArmReq-response>)))
  "Returns string type for a service object of type '<mArmReq-response>"
  "swarm_center/mArmReqResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mArmReq-response)))
  "Returns string type for a service object of type 'mArmReq-response"
  "swarm_center/mArmReqResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mArmReq-response>)))
  "Returns md5sum for a message object of type '<mArmReq-response>"
  "cf33071162f00c3ef34465803c52a01d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mArmReq-response)))
  "Returns md5sum for a message object of type 'mArmReq-response"
  "cf33071162f00c3ef34465803c52a01d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mArmReq-response>)))
  "Returns full string definition for message of type '<mArmReq-response>"
  (cl:format cl:nil "bool b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mArmReq-response)))
  "Returns full string definition for message of type 'mArmReq-response"
  (cl:format cl:nil "bool b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mArmReq-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mArmReq-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mArmReq-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mArmReq)))
  'mArmReq-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mArmReq)))
  'mArmReq-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mArmReq)))
  "Returns string type for a service object of type '<mArmReq>"
  "swarm_center/mArmReq")