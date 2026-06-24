;; Auto-generated. Do not edit!


(when (boundp 'target_detector::MarkerPixelArray)
  (if (not (find-package "TARGET_DETECTOR"))
    (make-package "TARGET_DETECTOR"))
  (shadow 'MarkerPixelArray (find-package "TARGET_DETECTOR")))
(unless (find-package "TARGET_DETECTOR::MARKERPIXELARRAY")
  (make-package "TARGET_DETECTOR::MARKERPIXELARRAY"))

(in-package "ROS")
;;//! \htmlinclude MarkerPixelArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass target_detector::MarkerPixelArray
  :super ros::object
  :slots (_header _markers ))

(defmethod target_detector::MarkerPixelArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:markers __markers) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _markers __markers)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:markers
   (&rest __markers)
   (if (keywordp (car __markers))
       (send* _markers __markers)
     (progn
       (if __markers (setq _markers (car __markers)))
       _markers)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; target_detector/MarkerPixel[] _markers
    (apply #'+ (send-all _markers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; target_detector/MarkerPixel[] _markers
     (write-long (length _markers) s)
     (dolist (elem _markers)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; target_detector/MarkerPixel[] _markers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _markers (let (r) (dotimes (i n) (push (instance target_detector::MarkerPixel :init) r)) r))
     (dolist (elem- _markers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get target_detector::MarkerPixelArray :md5sum-) "44de9ad6def1705d14c002806a70e9db")
(setf (get target_detector::MarkerPixelArray :datatype-) "target_detector/MarkerPixelArray")
(setf (get target_detector::MarkerPixelArray :definition-)
      "std_msgs/Header header
MarkerPixel[] markers

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: target_detector/MarkerPixel
# AR 码像素坐标
uint32 id          # 标签 ID
float32 u          # 像素列坐标 (x)
float32 v          # 像素行坐标 (y)

")



(provide :target_detector/MarkerPixelArray "44de9ad6def1705d14c002806a70e9db")


