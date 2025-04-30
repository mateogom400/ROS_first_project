;; Auto-generated. Do not edit!


(when (boundp 'first_project::sector_time)
  (if (not (find-package "FIRST_PROJECT"))
    (make-package "FIRST_PROJECT"))
  (shadow 'sector_time (find-package "FIRST_PROJECT")))
(unless (find-package "FIRST_PROJECT::SECTOR_TIME")
  (make-package "FIRST_PROJECT::SECTOR_TIME"))

(in-package "ROS")
;;//! \htmlinclude sector_time.msg.html


(defclass first_project::sector_time
  :super ros::object
  :slots (_current_sector _current_sector_time _current_sector_mean_speed ))

(defmethod first_project::sector_time
  (:init
   (&key
    ((:current_sector __current_sector) 0)
    ((:current_sector_time __current_sector_time) 0.0)
    ((:current_sector_mean_speed __current_sector_mean_speed) 0.0)
    )
   (send-super :init)
   (setq _current_sector (round __current_sector))
   (setq _current_sector_time (float __current_sector_time))
   (setq _current_sector_mean_speed (float __current_sector_mean_speed))
   self)
  (:current_sector
   (&optional __current_sector)
   (if __current_sector (setq _current_sector __current_sector)) _current_sector)
  (:current_sector_time
   (&optional __current_sector_time)
   (if __current_sector_time (setq _current_sector_time __current_sector_time)) _current_sector_time)
  (:current_sector_mean_speed
   (&optional __current_sector_mean_speed)
   (if __current_sector_mean_speed (setq _current_sector_mean_speed __current_sector_mean_speed)) _current_sector_mean_speed)
  (:serialization-length
   ()
   (+
    ;; int32 _current_sector
    4
    ;; float64 _current_sector_time
    8
    ;; float64 _current_sector_mean_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _current_sector
       (write-long _current_sector s)
     ;; float64 _current_sector_time
       (sys::poke _current_sector_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _current_sector_mean_speed
       (sys::poke _current_sector_mean_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _current_sector
     (setq _current_sector (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _current_sector_time
     (setq _current_sector_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _current_sector_mean_speed
     (setq _current_sector_mean_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get first_project::sector_time :md5sum-) "1faea2573a9ef916b33bea5b9868fa6d")
(setf (get first_project::sector_time :datatype-) "first_project/sector_time")
(setf (get first_project::sector_time :definition-)
      "int32 current_sector
float64 current_sector_time
float64 current_sector_mean_speed


")



(provide :first_project/sector_time "1faea2573a9ef916b33bea5b9868fa6d")


