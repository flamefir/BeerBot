;; Auto-generated. Do not edit!


(when (boundp 'my_pkg::Camera)
  (if (not (find-package "MY_PKG"))
    (make-package "MY_PKG"))
  (shadow 'Camera (find-package "MY_PKG")))
(unless (find-package "MY_PKG::CAMERA")
  (make-package "MY_PKG::CAMERA"))

(in-package "ROS")
;;//! \htmlinclude Camera.msg.html


(defclass my_pkg::Camera
  :super ros::object
  :slots (_message ))

(defmethod my_pkg::Camera
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get my_pkg::Camera :md5sum-) "5f003d6bcc824cbd51361d66d8e4f76c")
(setf (get my_pkg::Camera :datatype-) "my_pkg/Camera")
(setf (get my_pkg::Camera :definition-)
      "string message

")



(provide :my_pkg/Camera "5f003d6bcc824cbd51361d66d8e4f76c")


