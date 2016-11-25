

(ros::load-ros-manifest "roseus")
(ros::roseus "checkerboard_listner")

(defun sub-checkerboard (msg)
  (ros::ros-info "hoge")
  )

(ros::subscribe "/checkerboard_detector0/objectdetection_pose" geometry_msgs::PoseStamped #'sub-checkerboard)

(ros::spin)