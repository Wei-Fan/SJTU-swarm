; Auto-generated. Do not edit!


(cl:in-package swarm_center-srv)


;//! \htmlinclude mCPPReq-request.msg.html

(cl:defclass <mCPPReq-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mCPPReq-request (<mCPPReq-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mCPPReq-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mCPPReq-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm_center-srv:<mCPPReq-request> is deprecated: use swarm_center-srv:mCPPReq-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <mCPPReq-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm_center-srv:a-val is deprecated.  Use swarm_center-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mCPPReq-request>) ostream)
  "Serializes a message object of type '<mCPPReq-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'a) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mCPPReq-request>) istream)
  "Deserializes a message object of type '<mCPPReq-request>"
    (cl:setf (cl:slot-value msg 'a) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mCPPReq-request)))
  "Returns md5sum for a message object of type 'mCPPReq-request"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mCPPReq-request>)))
  "Returns full string definition for message of type '<mCPPReq-request>"
  (cl:format cl:nil "bool a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mCPPReq-request)))
  "Returns full string definition for message of type 'mCPPReq-request"
  (cl:format cl:nil "bool a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mCPPReq-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mCPPReq-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mCPPReq-request
    (cl:cons ':a (a msg))
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
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mCPPReq-response)))
  "Returns md5sum for a message object of type 'mCPPReq-response"
  "81f01bfd9a951b1adf9102125874ff5b")
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