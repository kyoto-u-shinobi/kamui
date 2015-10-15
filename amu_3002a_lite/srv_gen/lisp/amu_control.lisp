; Auto-generated. Do not edit!


(cl:in-package amu_3002a_lite-srv)


;//! \htmlinclude amu_control-request.msg.html

(cl:defclass <amu_control-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass amu_control-request (<amu_control-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <amu_control-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'amu_control-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amu_3002a_lite-srv:<amu_control-request> is deprecated: use amu_3002a_lite-srv:amu_control-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <amu_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amu_3002a_lite-srv:command-val is deprecated.  Use amu_3002a_lite-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <amu_control-request>) ostream)
  "Serializes a message object of type '<amu_control-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <amu_control-request>) istream)
  "Deserializes a message object of type '<amu_control-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<amu_control-request>)))
  "Returns string type for a service object of type '<amu_control-request>"
  "amu_3002a_lite/amu_controlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amu_control-request)))
  "Returns string type for a service object of type 'amu_control-request"
  "amu_3002a_lite/amu_controlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<amu_control-request>)))
  "Returns md5sum for a message object of type '<amu_control-request>"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'amu_control-request)))
  "Returns md5sum for a message object of type 'amu_control-request"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<amu_control-request>)))
  "Returns full string definition for message of type '<amu_control-request>"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'amu_control-request)))
  "Returns full string definition for message of type 'amu_control-request"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <amu_control-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <amu_control-request>))
  "Converts a ROS message object to a list"
  (cl:list 'amu_control-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude amu_control-response.msg.html

(cl:defclass <amu_control-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass amu_control-response (<amu_control-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <amu_control-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'amu_control-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amu_3002a_lite-srv:<amu_control-response> is deprecated: use amu_3002a_lite-srv:amu_control-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <amu_control-response>) ostream)
  "Serializes a message object of type '<amu_control-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <amu_control-response>) istream)
  "Deserializes a message object of type '<amu_control-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<amu_control-response>)))
  "Returns string type for a service object of type '<amu_control-response>"
  "amu_3002a_lite/amu_controlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amu_control-response)))
  "Returns string type for a service object of type 'amu_control-response"
  "amu_3002a_lite/amu_controlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<amu_control-response>)))
  "Returns md5sum for a message object of type '<amu_control-response>"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'amu_control-response)))
  "Returns md5sum for a message object of type 'amu_control-response"
  "481ac5a494c3140a2539020bd74c82c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<amu_control-response>)))
  "Returns full string definition for message of type '<amu_control-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'amu_control-response)))
  "Returns full string definition for message of type 'amu_control-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <amu_control-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <amu_control-response>))
  "Converts a ROS message object to a list"
  (cl:list 'amu_control-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'amu_control)))
  'amu_control-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'amu_control)))
  'amu_control-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amu_control)))
  "Returns string type for a service object of type '<amu_control>"
  "amu_3002a_lite/amu_control")