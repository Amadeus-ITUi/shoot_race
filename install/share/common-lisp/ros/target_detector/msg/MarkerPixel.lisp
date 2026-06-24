; Auto-generated. Do not edit!


(cl:in-package target_detector-msg)


;//! \htmlinclude MarkerPixel.msg.html

(cl:defclass <MarkerPixel> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0))
)

(cl:defclass MarkerPixel (<MarkerPixel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkerPixel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkerPixel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name target_detector-msg:<MarkerPixel> is deprecated: use target_detector-msg:MarkerPixel instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <MarkerPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detector-msg:id-val is deprecated.  Use target_detector-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <MarkerPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detector-msg:u-val is deprecated.  Use target_detector-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <MarkerPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detector-msg:v-val is deprecated.  Use target_detector-msg:v instead.")
  (v m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkerPixel>) ostream)
  "Serializes a message object of type '<MarkerPixel>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkerPixel>) istream)
  "Deserializes a message object of type '<MarkerPixel>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkerPixel>)))
  "Returns string type for a message object of type '<MarkerPixel>"
  "target_detector/MarkerPixel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkerPixel)))
  "Returns string type for a message object of type 'MarkerPixel"
  "target_detector/MarkerPixel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkerPixel>)))
  "Returns md5sum for a message object of type '<MarkerPixel>"
  "c6d7f7e63e402c2bfb95b9659cdc90fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkerPixel)))
  "Returns md5sum for a message object of type 'MarkerPixel"
  "c6d7f7e63e402c2bfb95b9659cdc90fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkerPixel>)))
  "Returns full string definition for message of type '<MarkerPixel>"
  (cl:format cl:nil "# AR 码像素坐标~%uint32 id          # 标签 ID~%float32 u          # 像素列坐标 (x)~%float32 v          # 像素行坐标 (y)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkerPixel)))
  "Returns full string definition for message of type 'MarkerPixel"
  (cl:format cl:nil "# AR 码像素坐标~%uint32 id          # 标签 ID~%float32 u          # 像素列坐标 (x)~%float32 v          # 像素行坐标 (y)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkerPixel>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkerPixel>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkerPixel
    (cl:cons ':id (id msg))
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
))
