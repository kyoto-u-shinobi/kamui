; Auto-generated. Do not edit!


(cl:in-package amu_3002a_lite-srv)


;//! \htmlinclude amu_operator-request.msg.html

(cl:defclass <amu_operator-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass amu_operator-request (<amu_operator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <amu_operator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'amu_operator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amu_3002a_lite-srv:<amu_operator-request> is deprecated: use amu_3002a_lite-srv:amu_operator-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <amu_operator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amu_3002a_lite-srv:command-val is deprecated.  Use amu_3002a_lite-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <amu_operator-request>) ostream)
  "Serializes a message object of type '<amu_operator-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <amu_operator-request>) istream)
  "Deserializes a message object of type '<amu_operator-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<amu_operator-request>)))
  "Returns string type for a service object of type '<amu_operator-request>"
  "amu_3002a_lite/amu_operatorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amu_operator-request)))
  "Returns string type for a service object of type 'amu_operator-request"
  "amu_3002a_lite/amu_operatorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<amu_operator-request>)))
  "Returns md5sum for a message object of type '<amu_operator-request>"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'amu_operator-request)))
  "Returns md5sum for a message object of type 'amu_operator-request"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<amu_operator-request>)))
  "Returns full string definition for message of type '<amu_operator-request>"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'amu_operator-request)))
  "Returns full string definition for message of type 'amu_operator-request"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <amu_operator-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <amu_operator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'amu_operator-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude amu_operator-response.msg.html

(cl:defclass <amu_operator-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass amu_operator-response (<amu_operator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <amu_operator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'amu_operator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amu_3002a_lite-srv:<amu_operator-response> is deprecated: use amu_3002a_lite-srv:amu_operator-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <amu_operator-response>) ostream)
  "Serializes a message object of type '<amu_operator-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <amu_operator-response>) istream)
  "Deserializes a message object of type '<amu_operator-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<amu_operator-response>)))
  "Returns string type for a service object of type '<amu_operator-response>"
  "amu_3002a_lite/amu_operatorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amu_operator-response)))
  "Returns string type for a service object of type 'amu_operator-response"
  "amu_3002a_lite/amu_operatorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<amu_operator-response>)))
  "Returns md5sum for a message object of type '<amu_operator-response>"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'amu_operator-response)))
  "Returns md5sum for a message object of type 'amu_operator-response"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<amu_operator-response>)))
  "Returns full string definition for message of type '<amu_operator-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'amu_operator-response)))
  "Returns full string definition for message of type 'amu_operator-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <amu_operator-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <amu_operator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'amu_operator-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'amu_operator)))
  'amu_operator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'amu_operator)))
  'amu_operator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amu_operator)))
  "Returns string type for a service object of type '<amu_operator>"
  "amu_3002a_lite/amu_operator")