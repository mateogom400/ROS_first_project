; Auto-generated. Do not edit!


(cl:in-package first_project-msg)


;//! \htmlinclude sector_time.msg.html

(cl:defclass <sector_time> (roslisp-msg-protocol:ros-message)
  ((current_sector
    :reader current_sector
    :initarg :current_sector
    :type cl:integer
    :initform 0)
   (current_sector_time
    :reader current_sector_time
    :initarg :current_sector_time
    :type cl:float
    :initform 0.0)
   (current_sector_mean_speed
    :reader current_sector_mean_speed
    :initarg :current_sector_mean_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass sector_time (<sector_time>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sector_time>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sector_time)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name first_project-msg:<sector_time> is deprecated: use first_project-msg:sector_time instead.")))

(cl:ensure-generic-function 'current_sector-val :lambda-list '(m))
(cl:defmethod current_sector-val ((m <sector_time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader first_project-msg:current_sector-val is deprecated.  Use first_project-msg:current_sector instead.")
  (current_sector m))

(cl:ensure-generic-function 'current_sector_time-val :lambda-list '(m))
(cl:defmethod current_sector_time-val ((m <sector_time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader first_project-msg:current_sector_time-val is deprecated.  Use first_project-msg:current_sector_time instead.")
  (current_sector_time m))

(cl:ensure-generic-function 'current_sector_mean_speed-val :lambda-list '(m))
(cl:defmethod current_sector_mean_speed-val ((m <sector_time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader first_project-msg:current_sector_mean_speed-val is deprecated.  Use first_project-msg:current_sector_mean_speed instead.")
  (current_sector_mean_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sector_time>) ostream)
  "Serializes a message object of type '<sector_time>"
  (cl:let* ((signed (cl:slot-value msg 'current_sector)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current_sector_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current_sector_mean_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sector_time>) istream)
  "Deserializes a message object of type '<sector_time>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_sector) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_sector_time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_sector_mean_speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sector_time>)))
  "Returns string type for a message object of type '<sector_time>"
  "first_project/sector_time")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sector_time)))
  "Returns string type for a message object of type 'sector_time"
  "first_project/sector_time")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sector_time>)))
  "Returns md5sum for a message object of type '<sector_time>"
  "1faea2573a9ef916b33bea5b9868fa6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sector_time)))
  "Returns md5sum for a message object of type 'sector_time"
  "1faea2573a9ef916b33bea5b9868fa6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sector_time>)))
  "Returns full string definition for message of type '<sector_time>"
  (cl:format cl:nil "int32 current_sector~%float64 current_sector_time~%float64 current_sector_mean_speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sector_time)))
  "Returns full string definition for message of type 'sector_time"
  (cl:format cl:nil "int32 current_sector~%float64 current_sector_time~%float64 current_sector_mean_speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sector_time>))
  (cl:+ 0
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sector_time>))
  "Converts a ROS message object to a list"
  (cl:list 'sector_time
    (cl:cons ':current_sector (current_sector msg))
    (cl:cons ':current_sector_time (current_sector_time msg))
    (cl:cons ':current_sector_mean_speed (current_sector_mean_speed msg))
))
