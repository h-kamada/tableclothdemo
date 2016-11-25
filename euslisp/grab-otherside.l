
;(load "package://tableclothdemo/euslisp/demo.l")
(load "demo.l")
(ros::roseus "dummy-grab-otherside")
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

;; otherside setting
(ros::subscribe "raw_tablecloth_bb" jsk_recognition_msgs::BoundingBoxArray #'sub-raw-tablecloth-bb)
(ros::subscribe "/move_to_table/where" std_msgs::String #'sub-move-to-table-where)
(ros::advertise "/tmp_pos" geometry_msgs::PoseArray 4)

