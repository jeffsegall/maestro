; Auto-generated. Do not edit!


(cl:in-package hubomsg-msg)


;//! \htmlinclude CanMessage.msg.html

(cl:defclass <CanMessage> (roslisp-msg-protocol:ros-message)
  ((mType
    :reader mType
    :initarg :mType
    :type cl:fixnum
    :initform 0)
   (cmdType
    :reader cmdType
    :initarg :cmdType
    :type cl:fixnum
    :initform 0)
   (bno
    :reader bno
    :initarg :bno
    :type cl:fixnum
    :initform 0)
   (r1
    :reader r1
    :initarg :r1
    :type cl:integer
    :initform 0)
   (r2
    :reader r2
    :initarg :r2
    :type cl:integer
    :initform 0)
   (r3
    :reader r3
    :initarg :r3
    :type cl:integer
    :initform 0)
   (r4
    :reader r4
    :initarg :r4
    :type cl:integer
    :initform 0)
   (r5
    :reader r5
    :initarg :r5
    :type cl:integer
    :initform 0)
   (r6
    :reader r6
    :initarg :r6
    :type cl:integer
    :initform 0)
   (r7
    :reader r7
    :initarg :r7
    :type cl:integer
    :initform 0)
   (r8
    :reader r8
    :initarg :r8
    :type cl:integer
    :initform 0))
)

(cl:defclass CanMessage (<CanMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CanMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CanMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hubomsg-msg:<CanMessage> is deprecated: use hubomsg-msg:CanMessage instead.")))

(cl:ensure-generic-function 'mType-val :lambda-list '(m))
(cl:defmethod mType-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:mType-val is deprecated.  Use hubomsg-msg:mType instead.")
  (mType m))

(cl:ensure-generic-function 'cmdType-val :lambda-list '(m))
(cl:defmethod cmdType-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:cmdType-val is deprecated.  Use hubomsg-msg:cmdType instead.")
  (cmdType m))

(cl:ensure-generic-function 'bno-val :lambda-list '(m))
(cl:defmethod bno-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:bno-val is deprecated.  Use hubomsg-msg:bno instead.")
  (bno m))

(cl:ensure-generic-function 'r1-val :lambda-list '(m))
(cl:defmethod r1-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r1-val is deprecated.  Use hubomsg-msg:r1 instead.")
  (r1 m))

(cl:ensure-generic-function 'r2-val :lambda-list '(m))
(cl:defmethod r2-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r2-val is deprecated.  Use hubomsg-msg:r2 instead.")
  (r2 m))

(cl:ensure-generic-function 'r3-val :lambda-list '(m))
(cl:defmethod r3-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r3-val is deprecated.  Use hubomsg-msg:r3 instead.")
  (r3 m))

(cl:ensure-generic-function 'r4-val :lambda-list '(m))
(cl:defmethod r4-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r4-val is deprecated.  Use hubomsg-msg:r4 instead.")
  (r4 m))

(cl:ensure-generic-function 'r5-val :lambda-list '(m))
(cl:defmethod r5-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r5-val is deprecated.  Use hubomsg-msg:r5 instead.")
  (r5 m))

(cl:ensure-generic-function 'r6-val :lambda-list '(m))
(cl:defmethod r6-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r6-val is deprecated.  Use hubomsg-msg:r6 instead.")
  (r6 m))

(cl:ensure-generic-function 'r7-val :lambda-list '(m))
(cl:defmethod r7-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r7-val is deprecated.  Use hubomsg-msg:r7 instead.")
  (r7 m))

(cl:ensure-generic-function 'r8-val :lambda-list '(m))
(cl:defmethod r8-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hubomsg-msg:r8-val is deprecated.  Use hubomsg-msg:r8 instead.")
  (r8 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CanMessage>) ostream)
  "Serializes a message object of type '<CanMessage>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmdType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bno)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'r1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r8)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CanMessage>) istream)
  "Deserializes a message object of type '<CanMessage>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmdType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bno)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r5) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r6) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r7) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r8) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CanMessage>)))
  "Returns string type for a message object of type '<CanMessage>"
  "hubomsg/CanMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CanMessage)))
  "Returns string type for a message object of type 'CanMessage"
  "hubomsg/CanMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CanMessage>)))
  "Returns md5sum for a message object of type '<CanMessage>"
  "8a0a761f0ea023e60de69eb361380cf7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CanMessage)))
  "Returns md5sum for a message object of type 'CanMessage"
  "8a0a761f0ea023e60de69eb361380cf7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CanMessage>)))
  "Returns full string definition for message of type '<CanMessage>"
  (cl:format cl:nil "uint16 mType~%uint8 cmdType~%uint8 bno~%int32 r1~%int32 r2~%int32 r3~%int32 r4~%int32 r5~%int32 r6~%int32 r7~%int32 r8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CanMessage)))
  "Returns full string definition for message of type 'CanMessage"
  (cl:format cl:nil "uint16 mType~%uint8 cmdType~%uint8 bno~%int32 r1~%int32 r2~%int32 r3~%int32 r4~%int32 r5~%int32 r6~%int32 r7~%int32 r8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CanMessage>))
  (cl:+ 0
     2
     1
     1
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CanMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'CanMessage
    (cl:cons ':mType (mType msg))
    (cl:cons ':cmdType (cmdType msg))
    (cl:cons ':bno (bno msg))
    (cl:cons ':r1 (r1 msg))
    (cl:cons ':r2 (r2 msg))
    (cl:cons ':r3 (r3 msg))
    (cl:cons ':r4 (r4 msg))
    (cl:cons ':r5 (r5 msg))
    (cl:cons ':r6 (r6 msg))
    (cl:cons ':r7 (r7 msg))
    (cl:cons ':r8 (r8 msg))
))
