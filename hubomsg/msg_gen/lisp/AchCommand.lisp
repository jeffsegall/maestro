; Auto-generated. Do not edit!


(cl:in-package hubomsg-msg)


;//! \htmlinclude AchCommand.msg.html

(cl:defclass <AchCommand> (roslisp-msg-protocol:ros-message)
  ((commandName
    :reader commandName
    :initarg :commandName
    :type cl:string
    :initform "")
   (jointName
    :reader jointName
    :initarg :jointName
    :type cl:string
    :initform ""))
)

(cl:defclass AchCommand (<AchCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AchCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AchCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hubomsg-msg:<AchCommand> is deprecated: use hubomsg-msg:AchCommand instead.")))

(cl:ensure-generic-function 'commandName-val :lambda-list '(m))
(cl:defmethod commandName-val ((m <AchCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:commandName-val is deprecated.  Use hubomsg-msg:commandName instead.")
  (commandName m))

(cl:ensure-generic-function 'jointName-val :lambda-list '(m))
(cl:defmethod jointName-val ((m <AchCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:jointName-val is deprecated.  Use hubomsg-msg:jointName instead.")
  (jointName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AchCommand>) ostream)
  "Serializes a message object of type '<AchCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'commandName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'commandName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'jointName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'jointName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AchCommand>) istream)
  "Deserializes a message object of type '<AchCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'commandName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'commandName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jointName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'jointName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AchCommand>)))
  "Returns string type for a message object of type '<AchCommand>"
  "hubomsg/AchCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AchCommand)))
  "Returns string type for a message object of type 'AchCommand"
  "hubomsg/AchCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AchCommand>)))
  "Returns md5sum for a message object of type '<AchCommand>"
  "70a66f2cb750752eb65c63fd2693f894")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AchCommand)))
  "Returns md5sum for a message object of type 'AchCommand"
  "70a66f2cb750752eb65c63fd2693f894")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AchCommand>)))
  "Returns full string definition for message of type '<AchCommand>"
  (cl:format cl:nil "string commandName~%string jointName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AchCommand)))
  "Returns full string definition for message of type 'AchCommand"
  (cl:format cl:nil "string commandName~%string jointName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AchCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'commandName))
     4 (cl:length (cl:slot-value msg 'jointName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AchCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'AchCommand
    (cl:cons ':commandName (commandName msg))
    (cl:cons ':jointName (jointName msg))
))
