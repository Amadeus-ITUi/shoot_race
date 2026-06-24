; Auto-generated. Do not edit!


(cl:in-package target_detector-msg)


;//! \htmlinclude MarkerPixelArray.msg.html

(cl:defclass <MarkerPixelArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (markers
    :reader markers
    :initarg :markers
    :type (cl:vector target_detector-msg:MarkerPixel)
   :initform (cl:make-array 0 :element-type 'target_detector-msg:MarkerPixel :initial-element (cl:make-instance 'target_detector-msg:MarkerPixel))))
)

(cl:defclass MarkerPixelArray (<MarkerPixelArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkerPixelArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkerPixelArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name target_detector-msg:<MarkerPixelArray> is deprecated: use target_detector-msg:MarkerPixelArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MarkerPixelArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detector-msg:header-val is deprecated.  Use target_detector-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'markers-val :lambda-list '(m))
(cl:defmethod markers-val ((m <MarkerPixelArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detector-msg:markers-val is deprecated.  Use target_detector-msg:markers instead.")
  (markers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkerPixelArray>) ostream)
  "Serializes a message object of type '<MarkerPixelArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'markers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkerPixelArray>) istream)
  "Deserializes a message object of type '<MarkerPixelArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'markers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'markers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'target_detector-msg:MarkerPixel))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkerPixelArray>)))
  "Returns string type for a message object of type '<MarkerPixelArray>"
  "target_detector/MarkerPixelArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkerPixelArray)))
  "Returns string type for a message object of type 'MarkerPixelArray"
  "target_detector/MarkerPixelArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkerPixelArray>)))
  "Returns md5sum for a message object of type '<MarkerPixelArray>"
  "44de9ad6def1705d14c002806a70e9db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkerPixelArray)))
  "Returns md5sum for a message object of type 'MarkerPixelArray"
  "44de9ad6def1705d14c002806a70e9db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkerPixelArray>)))
  "Returns full string definition for message of type '<MarkerPixelArray>"
  (cl:format cl:nil "std_msgs/Header header~%MarkerPixel[] markers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: target_detector/MarkerPixel~%# AR 码像素坐标~%uint32 id          # 标签 ID~%float32 u          # 像素列坐标 (x)~%float32 v          # 像素行坐标 (y)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkerPixelArray)))
  "Returns full string definition for message of type 'MarkerPixelArray"
  (cl:format cl:nil "std_msgs/Header header~%MarkerPixel[] markers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: target_detector/MarkerPixel~%# AR 码像素坐标~%uint32 id          # 标签 ID~%float32 u          # 像素列坐标 (x)~%float32 v          # 像素行坐标 (y)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkerPixelArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkerPixelArray>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkerPixelArray
    (cl:cons ':header (header msg))
    (cl:cons ':markers (markers msg))
))
