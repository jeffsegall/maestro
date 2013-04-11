; Auto-generated. Do not edit!


(cl:in-package hubomsg-msg)


;//! \htmlinclude HuboCmd.msg.html

(cl:defclass <HuboCmd> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type cl:integer
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass HuboCmd (<HuboCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HuboCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HuboCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hubomsg-msg:<HuboCmd> is deprecated: use hubomsg-msg:HuboCmd instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <HuboCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:joint-val is deprecated.  Use hubomsg-msg:joint instead.")
  (joint m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <HuboCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:angle-val is deprecated.  Use hubomsg-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <HuboCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:msg-val is deprecated.  Use hubomsg-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HuboCmd>) ostream)
  "Serializes a message object of type '<HuboCmd>"
  (cl:let* ((signed (cl:slot-value msg 'joint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HuboCmd>) istream)
  "Deserializes a message object of type '<HuboCmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HuboCmd>)))
  "Returns string type for a message object of type '<HuboCmd>"
  "hubomsg/HuboCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HuboCmd)))
  "Returns string type for a message object of type 'HuboCmd"
  "hubomsg/HuboCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HuboCmd>)))
  "Returns md5sum for a message object of type '<HuboCmd>"
  "d5db4a16ad98b99e078cccd0e81d4786")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HuboCmd)))
  "Returns md5sum for a message object of type 'HuboCmd"
  "d5db4a16ad98b99e078cccd0e81d4786")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HuboCmd>)))
  "Returns full string definition for message of type '<HuboCmd>"
  (cl:format cl:nil "int64 joint~%float64 angle~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HuboCmd)))
  "Returns full string definition for message of type 'HuboCmd"
  (cl:format cl:nil "int64 joint~%float64 angle~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HuboCmd>))
  (cl:+ 0
     8
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HuboCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'HuboCmd
    (cl:cons ':joint (joint msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':msg (msg msg))
))
