
(cl:in-package :asdf)

(defsystem "first_project-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sector_time" :depends-on ("_package_sector_time"))
    (:file "_package_sector_time" :depends-on ("_package"))
  ))