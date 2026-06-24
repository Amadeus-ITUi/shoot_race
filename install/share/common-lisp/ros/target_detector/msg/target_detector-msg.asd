
(cl:in-package :asdf)

(defsystem "target_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MarkerPixel" :depends-on ("_package_MarkerPixel"))
    (:file "_package_MarkerPixel" :depends-on ("_package"))
    (:file "MarkerPixelArray" :depends-on ("_package_MarkerPixelArray"))
    (:file "_package_MarkerPixelArray" :depends-on ("_package"))
  ))