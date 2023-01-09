
(cl:in-package :asdf)

(defsystem "my_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Camera" :depends-on ("_package_Camera"))
    (:file "_package_Camera" :depends-on ("_package"))
  ))