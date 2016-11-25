
;(load "package://tableclothdemo/euslisp/demo.l")
(load "demo.l")
(ros::roseus "grab-side")
;; common setting
(ros::subscribe "tablecloth_bb" jsk_recognition_msgs::BoundingBoxArray #'sub-tablecloth-bb)
(ros::subscribe "tablecloth_bb_fixed"
                jsk_recognition_msgs::BoundingBoxArray #'sub-tablecloth-bb-fixed)
(ros::subscribe "/down_left_corner" geometry_msgs::PoseArray #'sub-downleft)
(ros::subscribe "/down_right_corner" geometry_msgs::PoseArray #'sub-downright)
(ros::subscribe "/table_static" geometry_msgs::PoseArray #'sub-tablestatic)
(ros::advertise "/move_standing_pos" geometry_msgs::PoseArray 4)
(ros::advertise "/adjust_standing_pos" geometry_msgs::PoseArray 4)
;; (ros::subscribe "table_marker_array"visualization_msgs::MarkerArray #'sub-tablemarkerarray)

;; grabside setting
(ros::advertise "raw_tablecloth_bb" jsk_recognition_msgs::BoundingBoxArray 5)
(ros::advertise "tablecloth_height" std_msgs::Float32 5)
(ros::advertise "/move_to_table/where" std_msgs::String 5)
(ros::subscribe "checkerboard_pose" geometry_msgs::PoseStamped #'sub-checkerboard)

(ros::rate 1)
