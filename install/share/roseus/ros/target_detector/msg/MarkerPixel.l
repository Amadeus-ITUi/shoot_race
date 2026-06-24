;; Auto-generated. Do not edit!


(when (boundp 'target_detector::MarkerPixel)
  (if (not (find-package "TARGET_DETECTOR"))
    (make-package "TARGET_DETECTOR"))
  (shadow 'MarkerPixel (find-package "TARGET_DETECTOR")))
(unless (find-package "TARGET_DETECTOR::MARKERPIXEL")
  (make-package "TARGET_DETECTOR::MARKERPIXEL"))

(in-package "ROS")
;;//! \htmlinclude MarkerPixel.msg.html


(defclass target_detector::MarkerPixel
  :super ros::object
  :slots (_id _u _v ))

(defmethod target_detector::MarkerPixel
  (:init
   (&key
    ((:id __id) 0)
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _u (float __u))
   (setq _v (float __v))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; float32 _u
    4
    ;; float32 _v
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; float32 _u
       (sys::poke _u (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v
       (sys::poke _v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _u
     (setq _u (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v
     (setq _v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get target_detector::MarkerPixel :md5sum-) "c6d7f7e63e402c2bfb95b9659cdc90fc")
(setf (get target_detector::MarkerPixel :datatype-) "target_detector/MarkerPixel")
(setf (get target_detector::MarkerPixel :definition-)
      "# AR 码像素坐标
uint32 id          # 标签 ID
float32 u          # 像素列坐标 (x)
float32 v          # 像素行坐标 (y)

")



(provide :target_detector/MarkerPixel "c6d7f7e63e402c2bfb95b9659cdc90fc")


