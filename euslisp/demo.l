(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "tf2_msgs")

(setq *center-cds* (send (make-coords) :locate (float-vector 700 0 950)))
(setq *clothwidth* 460)
(setq *move-forward-dis* 50)
(setq *move-up-dis* 300)

(setq *checkerboard-pos* nil)
(setq *expected-cb-coords* (send (make-coords) :translate (float-vector 1140 -60 0)))
(setq *ratio* 1)
(setq *ratio* (* *ratio* 0.001))

(setq *l-coords* (send (send (send *center-cds* :copy-worldcoords)
                             :translate (float-vector 0 (/ *clothwidth* 2) 0))
                       :rotate (deg2rad 90) :x))
(setq *r-coords* (send (send (send *center-cds* :copy-worldcoords)
                             :translate (float-vector 0 (- (/ *clothwidth* 2)) 0))
                       :rotate (deg2rad -90) :x))

(defun sub-checkerboard (msg)
  (ros::ros-info "get-checkerboard")
  (setq *checkerboard-pos* msg))

(defun approach ()
  (let (cb-coords cb-field-coords stand-point-coords pr2-initial-pose)
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 300)
    (send *pr2* :head :neck-p :joint-angle 10)
    (setq pr2-initial-pose (send *pr2* :angle-vector))
    (send *ri* :angle-vector pr2-initial-pose)
    (ros::spin-once)
    (unless *checkerboard-pos* (return-from approach nil))
    (setq cb-coords (ros::tf-pose->coords (send *checkerboard-pos* :pose)))
    (send cb-coords :rotate (deg2rad -90) :y) ; raw arrow is pointing down. so rotate
    (setq cb-field-coords (send (make-coords) :locate (send cb-coords :pos)))
    (send cb-field-coords :rotate (elt (send (make-coords) :difference-rotation cb-coords) 2) :z)
    (send cb-field-coords :locate (float-vector 0 0 (- (elt (send cb-field-coords :pos) 2))))
    (setq stand-point-coords (send (make-coords) :move-to cb-field-coords))
    (send stand-point-coords :translate (v- (float-vector 0 0 0) (send *expected-cb-coords* :pos)))
    (ros::ros-info "cb-coords:~A" cb-coords)
    (ros::ros-info "stand-point:~A" stand-point-coords)
    (ros::ros-info "x:~A y:~A rot:~A"
                   (* *ratio* (elt (send stand-point-coords :pos) 0))
                   (* *ratio* (elt (send stand-point-coords :pos) 1))
                   (rad2deg (elt (send (make-coords) :difference-rotation stand-point-coords) 2))
                   )
    (send *ri* :go-pos-unsafe
          (* *ratio* (elt (send stand-point-coords :pos) 0))
          (* *ratio* (elt (send stand-point-coords :pos) 1))
          (rad2deg (elt (send (make-coords) :difference-rotation stand-point-coords) 2))
          )
    ))


(defun grab (&key (center-cds *center-cds*) (torso t))
  (let ((l-coords *l-coords*)
        (r-coords *r-coords*)
        pr2-initial-pose
        )
    ;; maybe needed when executing demo  ....
    (pub-height *tablecloth-height*)
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)
    (send *pr2* :reset-manip-pose)
    (when torso (send *pr2* :torso :waist-z :joint-angle 300))
    (send *pr2* :head :neck-p :joint-angle 10)
    (setq pr2-initial-pose (send *pr2* :angle-vector))

    (send *pr2* :angle-vector pr2-initial-pose)
    (send *ri* :angle-vector pr2-initial-pose)
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)

    (send *pr2* :angle-vector pr2-initial-pose)
    (send *pr2* :rarm :inverse-kinematics r-coords :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics l-coords :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (send *pr2* :angle-vector pr2-initial-pose)
    (send *pr2* :rarm :inverse-kinematics (send (send r-coords :copy-worldcoords)
                                                :translate (float-vector *move-forward-dis* 0 0))
          :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics (send (send l-coords :copy-worldcoords)
                                                :translate (float-vector *move-forward-dis* 0 0))
          :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp)
    (send *ri* :wait-interpolation)

    (send *pr2* :angle-vector pr2-initial-pose)
    (send *pr2* :rarm :inverse-kinematics r-coords :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics l-coords :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (send *pr2* :angle-vector pr2-initial-pose)
    (send *pr2* :rarm :inverse-kinematics (send (send r-coords :copy-worldcoords)
                                                :translate (float-vector 0 0 *move-up-dis*) :world)
          :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics (send (send l-coords :copy-worldcoords)
                                                :translate (float-vector 0 0 *move-up-dis*) :world)
          :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ))

(defun spread-tablecloth ()
  (let ((error-norm 0)
        (ev (float-vector 0 0 0 0 0 0 0 0 0 0 0 0 0 0))
        pr2-initial-pose
        (l-coords (send (send *l-coords* :copy-worldcoords) :translate (float-vector 0 0 *move-up-dis*) :world))
        (r-coords (send (send *r-coords* :copy-worldcoords) :translate (float-vector 0 0 *move-up-dis*) :world))
        )
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 300)
    (send *pr2* :head :neck-p :joint-angle 10)
    (send *pr2* :rarm :inverse-kinematics r-coords :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics l-coords :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (setq pr2-initial-pose (send *pr2* :angle-vector))
    (while (< error-norm 0.3)
      (let ((error-vector (send *ri* :error-vector))
            )
        (dotimes (x 14)
          (setf (elt ev x) (elt error-vector (+ 1 x))))
        (ros::ros-info "error-vector:~A" error-vector)
        (ros::ros-info "ev:~A" ev)
        (ros::ros-info "norm:~A" (norm ev))
        (setq error-norm (norm ev))
        (send *pr2* :rarm :inverse-kinematics
              (send r-coords :translate (float-vector 0 -15 0) :world)
              :rotation-axis t)
        (send *pr2* :larm :inverse-kinematics
              (send l-coords :translate (float-vector 0 15 0) :world)
              :rotation-axis t)
        (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
        (unix:usleep 300000)
        ))
    ;; Error detected. Little back
    (send *pr2* :rarm :inverse-kinematics
          (send r-coords :translate (float-vector 0 30 0) :world)
          :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics
          (send l-coords :translate (float-vector 0 -30 0) :world)
          :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ))

(defun raise-arms ()
  (let (l-cds
        r-cds
        cur-pos
        ret
        )
    (setq cur-pos (send *ri* :state :potentio-vector))
    (setq l-cds (send (send *pr2* :larm :end-coords) :copy-worldcoords))
    (setq r-cds (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (while t
      (send *pr2* :angle-vector (set-potentio-vector cur-pos :rarm :larm :torso))
      ;; above is needed to set head initial-pos
      (setq ret
            (list (send *pr2* :larm :inverse-kinematics
                        (send l-cds :translate (float-vector 0 0 15) :world) :rotation-axis nil)
                  (send *pr2* :rarm :inverse-kinematics
                        (send r-cds :translate (float-vector 0 0 15) :world) :rotation-axis nil)))
      (ros::ros-info "ret:~A" ret)
      (when (> (count nil ret) 0) ;; ik fail
        (send *ri* :wait-interpolation)
        (return-from raise-arms t))
      (send *ri* :angle-vector (set-potentio-vector (send *pr2* :angle-vector) :head) 500)
      )))

(setq *grab-center-cds* (send (make-coords) :locate (float-vector 600 0 800)))
(setq *grab-width* 900)

(setq *grab-center-cds* (send (make-coords) :locate (float-vector 500 0 650)))
(setq *grab-width* 900)

(setq *grab-center-cds* (send (make-coords) :locate (float-vector 500 0 650)))
(setq *grab-width* 900)

(setq *grab-center-cds* (send (make-coords) :locate (float-vector 530 0 600)))
(setq *grab-width* 700)

;; (defun grab-otherside (&optional center-coords width)
;;   (let (
;;         (cent-cds (cond (center-coords center-coords) (t *grab-center-cds*)))
;;         (wid (cond (width width) (t *grab-width*)))
;;         a b r-coords l-coords
;;         )
;;     (send *pr2* :reset-pose)
;;     (send *pr2* :torso :waist-z :joint-angle 50)
;;     (send *ri* :angle-vector (send *pr2* :angle-vector))
;;     (send *ri* :stop-grasp)
;;     (send *ri* :wait-interpolation)

;;     (setq r-coords (send (send cent-cds :copy-worldcoords)
;;                          :translate (float-vector 0 (- (/ wid 2)) 0)))
;;     (setq l-coords (send (send cent-cds :copy-worldcoords)
;;                          :translate (float-vector 0 (/ wid 2) 0)))
;;     (send l-coords :rotate (deg2rad -180) :x) ;; これ重要。これでほぼ左右対称になる
;;     (setq a (send (send (send (send r-coords :copy-worldcoords)
;;                               :rotate (deg2rad 90) :x) :rotate (deg2rad 90) :y)
;;                   :rotate (deg2rad -90) :y :world))
;;     (setq b (send (send (send (send (send l-coords :copy-worldcoords)
;;                                     :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :y)
;;                         :rotate (deg2rad 180) :z)
;;                   :rotate (deg2rad -90) :y :world))
;;     (send *pr2* :rarm :inverse-kinematics r-coords :rotation-axis t)
;;     (send *pr2* :larm :inverse-kinematics l-coords :rotation-axis t)
;;     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)

;;     (send *pr2* :rarm :inverse-kinematics
;;           (send (send a :copy-worldcoords) :translate (float-vector 0 -100 0) :world)
;;           :rotation-axis t)
;;     (send *pr2* :larm :inverse-kinematics
;;           (send (send b :copy-worldcoords) :translate (float-vector 0 100 0) :world)
;;           :rotation-axis t)
;;     (send *ri* :angle-vector (send *pr2* :angle-vector))
;;     (send *ri* :wait-interpolation)

;;     (send *pr2* :rarm :inverse-kinematics a :rotation-axis t)
;;     (send *pr2* :larm :inverse-kinematics b :rotation-axis t)
;;     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)

;;     ;; (send *pr2* :rarm :inverse-kinematics
;;     ;;       (send (send a :copy-worldcoords) :translate (float-vector 0 200 0) :world)
;;     ;;       :rotation-axis t)
;;     ;; (send *pr2* :larm :inverse-kinematics
;;     ;;       (send (send b :copy-worldcoords) :translate (float-vector 0 -200 0) :world)
;;     ;;       :rotation-axis t)
;;     ;; (send *ri* :angle-vector (send *pr2* :angle-vector))
;;     ;; (send *ri* :wait-interpolation)
;;     ))


(setq *tfl* (instance ros::transform-listener :init))
;; (defun pub-tablecloth-bb-old (&optional (dummy nil))
;;   (let (cloth-coords
;;         (posestamped (instance geometry_msgs::PoseStamped :init))
;;         (bb (instance jsk_recognition_msgs::BoundingBox :init))
;;         (bba (instance jsk_recognition_msgs::BoundingBoxArray :init))
;;         )
;;     (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;     (when dummy (send *pr2* :angle-vector (float-vector 300.0 37.52 -17.726 84.1502 -89.193 -202.294 -51.6361 187.15 -37.5784 -17.7457 -84.2607 -89.1877 202.244 -51.6387 -187.043 0.0 10.0)))
;;     (setq cloth-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
;;     (send cloth-coords :translate (float-vector 0 *width-offset* *height-offset*) :world)
;;     (send posestamped :pose (ros::coords->tf-pose cloth-coords))
;;     (send posestamped :header :stamp (ros::time-now))
;;     (send posestamped :header :frame_id "/base_footprint")

;;     ;; ;; transform from  /r_gripper to /map
;;     (cond
;;      ((null (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1))
;;       (ros::ros-error "could not solve tf. Force transfrom to /map")
;;       )
;;      (t
;;       (send posestamped :pose (ros::coords->tf-pose (send *tfl* :transform-pose "/map" posestamped)))
;;       ))
;;     ;; set to rosmsg
;;     (send posestamped :header :frame_id "/map")
;;     (send bba :header (send posestamped :header))
;;     (send bb :header (send posestamped :header))
;;     (send bb :pose (send posestamped :pose))
;;     (send bb :dimensions (instance geometry_msgs::Vector3 :init
;;                                    :x 0.7 :y *tablecloth-height* :z *tablecloth-width*))
;;     (send bba :boxes (list bb))
;;     ;; (print-ros-msg bba)
;;     (ros::publish "tablecloth_bb" bba)
;;     ;; (print-ros-msg bba)
;;     ))

(setq *height-offset* 25)
(setq *measure-deg-max* 62)
(defun measure-tablecloth-height ()
  (let (arm-center height)
    (search-lowest-point *measure-deg-max*)
    (unless *lowest-center*
      (measure-tablecloth-height))
    (setq arm-center
          (scale 0.5 (v+ (send (send *pr2* :rarm :end-coords) :worldpos)
                         (send (send *pr2* :larm :end-coords) :worldpos))))
    (setq height (+ (- (elt arm-center 2) (elt *lowest-center* 2)) *height-offset*))
    (ros::ros-info "arm-center:~A lowest-center:~A height:~A" arm-center *lowest-center* height)
    (pub-height height)
    (when (< (elt *lowest-center* 2) 100) ;; too low. Maybe ground!
      (measure-tablecloth-height)
    )))

(defun pub-height (height)
  (let (sf)
    (setq sf (instance std_msgs::Float32 :init :data height))
    (ros::publish "tablecloth_height" sf)
    ))

(defun sub-height (msg)
  (ros::ros-info "sub-height cb data:~A" (send msg :data))
  (setq *tablecloth-height* (send msg :data)))

(setq *tablecloth-height* 1500)
(defun pub-tablecloth-bb (&optional (dummy nil))
  (let (cloth-center-coords
        (posestamped (instance geometry_msgs::PoseStamped :init))
        (bb (instance jsk_recognition_msgs::BoundingBox :init))
        (bba (instance jsk_recognition_msgs::BoundingBoxArray :init))
        arms-distance
        bb-width
        bb-height
        bb-depth
        )
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (when dummy (send *pr2* :angle-vector (float-vector 300.0 37.52 -17.726 84.1502 -89.193 -202.294 -51.6361 187.15 -37.5784 -17.7457 -84.2607 -89.1877 202.244 -51.6387 -187.043 0.0 10.0)))
    (setq cloth-center-coords
          (send (make-coords) :translate
                (scale 0.5 (v+ (send (send *pr2* :rarm :end-coords) :worldpos)
                               (send (send *pr2* :larm :end-coords) :worldpos)))))

    (setq arms-distance
          (/ (norm (v- (send (send *pr2* :rarm :end-coords) :worldpos)
                       (send (send *pr2* :larm :end-coords) :worldpos))) 1000))
    (setq bb-width arms-distance)
    (setq bb-height (/ *tablecloth-height* 1000.0))
    (setq bb-depth 0.01)
    (send cloth-center-coords :translate (float-vector 0 0 (- (/ *tablecloth-height* 2))) :world)
    (send cloth-center-coords :rotate (deg2rad -90) :x :world)
    (send posestamped :pose (ros::coords->tf-pose cloth-center-coords))
    (send posestamped :header :stamp (ros::time-now))
    (send posestamped :header :frame_id "/base_footprint")
    ;; transform (wait-for-transform is very important for avoiding tf future past error)
    (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
      (ros::ros-error "could not solve tf. Return from pub-tablecloth-bb nil")
      (return-from pub-tablecloth-bb nil))
    (setq tmp-coords (send *tfl* :transform-pose "/map" posestamped))
    (unless tmp-coords
      (ros::ros-error "could not solve tf. Return from pub-tablecloth-bb nil")
      (return-from pub-tablecloth-bb nil))
    (send bb :header :stamp (send posestamped :header :stamp))
    (send bb :header :frame_id "/map")
    (send bb :pose (ros::coords->tf-pose tmp-coords))
    (send bb :dimensions (instance geometry_msgs::Vector3 :init
                                   :x bb-depth :y bb-height :z bb-width))
    (send bba :header (send bb :header))
    (send bba :boxes (list bb))
    (ros::ros-debug "publish tablecloth_bb")
    (ros::publish "raw_tablecloth_bb" bba)
    ))

(setq *tablecloth-bba-msg* nil)
(defun sub-tablecloth-bb (msg)
  ;; (ros::ros-info "sub-tablecloth-bb cb")
  (setq *tablecloth-bba-msg* msg)
  )

(defun pub-posearray (coords &optional (pub-name "hoge") (frame-id "/map"))
  (let ((posearray (instance geometry_msgs::PoseArray :init)))
    (ros::ros-info "pub-posearray \"~A\"(~A)" pub-name coords)
    (send posearray :header :stamp (ros::time-now))
    (send posearray :header :frame_id frame-id)
    (cond
     ((listp coords)
      (send posearray :poses (mapcar #'ros::coords->tf-pose coords)))
     (t
      (send posearray :poses (list (ros::coords->tf-pose coords)))))
    (ros::publish pub-name posearray)
    ))

(defun look-at-tablecloth-bb (&key (bottom nil))
  (let (bbam
        bb
        look-at-coords
        posestamped
        )
    (unless (check-tuckarm-pose)
      (pr2-tuckarm-pose))
    (ros::spin-once)
    (setq bbam *tablecloth-bb-fixed*)
    (when bbam (setq bb (car (send bbam :boxes))))
    (unless bb
      (send *pr2* :head :neck-p :joint-angle 30)
      (send *pr2* :head :neck-y :joint-angle 0) ;; default head position
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (ros::spin-once)
      (setq bbam *tablecloth-bb-fixed*)
      (when bbam (setq bb (car (send bbam :boxes))))
      (unless bb
        (ros::ros-error "Couldn't find bounding box. Are Rocon and clothdetecter fully working??")
        (return-from look-at-tablecloth-bb nil)
        ))
    (setq posestamped (instance geometry_msgs::PoseStamped :init))
    (send posestamped :pose (send bb :pose))
    (send posestamped :header (send bbam :header))
    (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
      (ros::ros-error "wait-for-transform failed. Return from look-at-tablecloth-bb nil")
      (return-from look-at-tablecloth-bb nil)
      )
    (setq look-at-coords (send *tfl* :transform-pose "/base_footprint" posestamped))
    (ros::ros-info "look-at-coords:~A" look-at-coords)
    (when bottom
      (send look-at-coords :translate
            (float-vector 0 0 (- (* 1000 (send bb :dimensions :y)))) :world)
      (ros::ros-info "look-at-coords(bottom):~A" look-at-coords)
      )
    (pub-posearray look-at-coords "/tmp_pos" "/base_footprint")
    (when look-at-coords (send *pr2* :head :look-at (send look-at-coords :worldpos)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ))

(defun look-at-raw-tablecloth-bb ()
  (let (bbam
        bb
        look-at-coords
        posestamped
        )
    (ros::spin-once)
    (setq bbam *tablecloth-bba-msg*)
    (unless bbam (return-from look-at-tablecloth-bb-fixed nil))
    (setq bb (car (send bbam :boxes)))
    (setq posestamped (instance geometry_msgs::PoseStamped :init))
    (send posestamped :pose (send bb :pose))
    (send posestamped :header (send bbam :header))
    (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
      (ros::ros-error "wait-for-transform failed. Return from look-at-raw-tablecloth-bb nil")
      (return-from look-at-raw-tablecloth-bb nil)
      )
    (setq look-at-coords (send *tfl* :transform-pose "/base_footprint" posestamped))
    (ros::ros-info "look-at-coords:~A" look-at-coords)
    (when look-at-coords (send *pr2* :head :look-at (send look-at-coords :worldpos)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ))

(setq *move-front-stay-back-distance* -1000)
(defun move-to-tablecloth-bb-front (&key (move t))
  (let (bbam
        bb
        look-at-coords
        bb-coords
        standing-coords
        posestamped
        z-axis-bb-rotation
        tmp-posestamped
        tmp-z-rotation
        )
    (ros::spin-once)
    (setq bbam *tablecloth-bba-msg*)
    (unless bbam (return-from move-to-tablecloth-bb-front nil))
    (setq bb (car (send bbam :boxes)))
    (setq posestamped (instance geometry_msgs::PoseStamped :init))
    (send posestamped :pose (send bb :pose))
    (send posestamped :header (send bbam :header))
    (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
      (ros::ros-error "wait-for-transform failed. Return from move-to-tablecloth-bb-front nil")
      (return-from move-to-tablecloth-bb-front nil)
      )
    (setq bb-coords (send *tfl* :transform-pose "/base_footprint" posestamped))
    (setq z-axis-bb-rotation
          (elt (send bb-coords :difference-rotation
                     (send (send (send (make-coords) :rotate (deg2rad -90) :y :world)
                                 :rotate (deg2rad -90) :x :world) :rotate (deg2rad 90) :z :world)) 1))
    (setq standing-coords (send bb-coords :orient 0 :z :world))
    (send standing-coords :rotate z-axis-bb-rotation :z :world)
    (send standing-coords :translate
          (float-vector *move-front-stay-back-distance* 0 (- (elt (send standing-coords :pos) 2))) :local)
    (setq standing-coords (send (make-coords) :move-to standing-coords))
    (pub-posearray standing-coords "/move_standing_pos" "/base_footprint")
    (when move (send *ri* :move-to standing-coords :frame-id "/base_footprint"))
    ))

(defun transform-coords (&optional (from "/map") (to "/base_footprint") (coords (make-coords))
                                   (stamp (ros::time-now)))
  (let (stmp posestamped tmp-coords)
    (cond
     (stamp
      (setq stmp stamp))
     (t
      (setq stmp (ros::time-now))))
    (unless (send *tfl* :wait-for-transform from to stmp 1)
      (ros::ros-error "wait-for-transform failed. Return from transform-coords nil")
      (return-from transform-coords nil))
    (setq posestamped (instance geometry_msgs::PoseStamped :init
                                :pose (ros::coords->tf-pose coords)))
    (send posestamped :header :stamp stmp)
    (send posestamped :header :frame_id from)
    (setq tmp-coords (send *tfl* :transform-pose to posestamped))
    (unless tmp-coords
      (ros::ros-error "could not solve tf. Return from transform-coords nil")
      (return-from transform-coords nil))
    tmp-coords
    ))

(setq *move-to-table-x-dis* 800)
(setq *move-to-table-y-dis* 1000)
(setq *move-to-table-margin* 500)

(defun move-to-table (&optional (setpos nil) (move nil) (moveto nil) (table-direction :x) (margin t))
  (let (standing-coords posestamped tmp-coords xdis ydis map-foot-deg map-table-deg
                        robot-plus-flag table-plus-flag robot-rot elt-num
                        )
    (when moveto
      (setq standing-coords (send (send (make-coords) :translate (float-vector 4750 6600 0) :world) :rotate (deg2rad -180) :z))
      (pub-posearray standing-coords "/adjust_standing_pos" "/map")
      (when move
        (send *ri* :move-to standing-coords :frame-id "/map")
        ))
    (ros::spin-once)
    (unless *table-static*
      (ros::ros-info "Couldn't find table static. Return from move-to-table nil")
      (return-from move-to-table nil))
    ;; Position with respect to the table is decided by the robot direction and table direction
    (setq map-foot-deg (rad2deg (elt (send (make-coords) :difference-rotation (transform-coords "/base_footprint" "/map")) 2)))
    (setq map-table-deg (rad2deg (elt (send (make-coords) :difference-rotation (ros::tf-pose->coords (car (send *table-static* :poses)))) 2)))
    (cond
     ((eq table-direction :y)
      (setq map-table-deg (+ map-table-deg 90)))
     ((eq table-direction :x)
      ;; do nothing
      )
     (t
      (ros::ros-error "table direction is :x or :y")
      (return-from move-to-table nil)
      ))
    (cond
     ((and (< -90 map-foot-deg) (< map-foot-deg 90)) ;; the robot is plus side to the map
      (setq robot-plus-flag 1))
     (t
      (setq robot-plus-flag -1)))
    (cond
     ((and (< -90 map-table-deg) (< map-table-deg 90)) ;; the table is plus side to the map
      (setq table-plus-flag 1))
     (t
      (setq table-plus-flag -1)))
    ;; set xdis and ydis according to the direction of table and robot
    (ros::ros-info "robot-flag:~A table-flag:~A table-direction:~A" robot-plus-flag table-plus-flag table-direction)
    (setq xdis (- (* (* robot-plus-flag table-plus-flag) *move-to-table-x-dis*)))
    (setq ydis (- (* table-plus-flag *move-to-table-y-dis*)))
    (when margin
      (setq ydis (- (* table-plus-flag (+ *move-to-table-y-dis* *move-to-table-margin*)))))
    (cond
     ((eq (* robot-plus-flag table-plus-flag) 1)
      (setq robot-rot 0))
     (t
      (setq robot-rot 180)))
    (when (eq table-direction :y)
      (setq xdis (* table-plus-flag *move-to-table-x-dis*))
      (when margin
        (setq xdis (* table-plus-flag (+ *move-to-table-x-dis* *move-to-table-margin*))))
      (setq ydis (- (* (* robot-plus-flag table-plus-flag) *move-to-table-y-dis*)))
      (setq robot-rot (+ robot-rot 90))
      )
    (setq standing-coords (ros::tf-pose->coords (car (send *table-static* :poses))))
    (cond
     ((eq table-direction :x)
      (send standing-coords :translate (float-vector xdis 0 0) :local))
     ((eq table-direction :y)
      (send standing-coords :translate (float-vector 0 ydis 0) :local)))
    (send standing-coords :translate
          (float-vector 0 0 (- (elt (send standing-coords :worldpos) 2))) :local)
    (send standing-coords :orient
          (elt (send (make-coords) :difference-rotation standing-coords) 2) :z :world)
    (unless setpos
      (cond
       ((eq table-direction :x)
        (send standing-coords :translate (float-vector 0 ydis 0) :local))
       ((eq table-direction :y)
        (send standing-coords :translate (float-vector xdis 0 0) :local))))
    (send standing-coords :rotate (deg2rad robot-rot) :z :local)
    (setq standing-coords (transform-coords "/map" "/base_footprint" standing-coords))
    (unless standing-coords
      (ros::ros-error "Return-from move-to-table nil")
      (return-from move-to-table nil))
    (pub-posearray standing-coords "/adjust_standing_pos" "/base_footprint")
    (ros::ros-info "go-pos-unsafe: ~A ~A ~A"
                   (* 0.001 (elt (send standing-coords :worldpos) 0))
                   (* 0.001 (elt (send standing-coords :worldpos) 1))
                   (rad2deg (elt (send (make-coords) :difference-rotation standing-coords) 2)))
    (when move
      (send *ri* :go-pos-unsafe
            (* 0.001 (elt (send standing-coords :worldpos) 0))
            (* 0.001 (elt (send standing-coords :worldpos) 1))
            (rad2deg (elt (send (make-coords) :difference-rotation standing-coords) 2)))
      )
    standing-coords
    ))

(setq *move-to-table-unsafe-moveback-dis* -1000)

(defun move-to-table-margin (&optional (table-direction :x))
  (pub-string "margin" "/move_to_table/where")
  (move-to-table nil nil nil table-direction t))
(defun move-to-table-around (&optional (table-direction :x))
  (pub-string "around" "/move_to_table/where")
  (move-to-table nil nil nil table-direction nil))
(defun move-to-table-setpos (&optional (table-direction :x))
  (pub-string "setpos" "/move_to_table/where")
  (move-to-table t nil nil table-direction nil))
(defun test-move-to-table (&optional (table-direction :x))
  (pub-string "test" "/move_to_table/where")
  (pub-posearray (list (move-to-table nil nil nil table-direction t)
                       (move-to-table nil nil nil table-direction nil)
                       (move-to-table t nil nil table-direction nil))
                 "/adjust_standing_pos" "base_footprint")
  )

(defun demo-move-to-table (&optional (table-direction :x))
  (dotimes (i 10)
    (pub-string "hoge" "/move_to_table/where")
    (unix:sleep 1))
  (pub-string "test" "/move_to_table/where")
  (pub-posearray (list (move-to-table nil nil nil table-direction t)
                       (move-to-table nil nil nil table-direction nil)
                       (move-to-table t nil nil table-direction nil))
                 "/adjust_standing_pos" "base_footprint")
  )

(setq *sub-move-to-table-table-direction* :x)
(defun sub-move-to-table-where (msg)
  (ros::ros-info "sub-move-to-table-where data:~A" (send msg :data))
  (cond
   ((string= (send msg :data) "margin")
    (ros::ros-info "hoge")
    (move-to-table nil nil nil  *sub-move-to-table-table-direction* t))
   ((string= (send msg :data) "around")
    (move-to-table nil nil nil  *sub-move-to-table-table-direction* nil))
   ((string= (send msg :data) "setpos")
    (move-to-table t nil nil  *sub-move-to-table-table-direction* nil))
   ((string= (send msg :data) "test")
    (pub-posearray (list (move-to-table nil nil nil  *sub-move-to-table-table-direction* t)
                         (move-to-table nil nil nil  *sub-move-to-table-table-direction* nil)
                         (move-to-table t nil nil  *sub-move-to-table-table-direction* nil))
                   "/adjust_standing_pos" "base_footprint")
    )
   (t
    (ros::ros-info "other")
    )))

(defun pub-string (data &optional (pub-name "/move_to_table/where"))
  (let (str)
    (ros::ros-info "pub-string data:~A" data)
    (setq str (instance std_msgs::String :init :data data))
    (ros::publish pub-name str)
    ))

(defun move-to-tablecloth-bb-front-old (&key (move t))
  (let (bbam
        cube
        bb
        standing-coords tmp-coords tmp2-coords tmp3-coords
        )
    (ros::spin-once)
    (print-ros-msg bbam)
    (print "##############################################")
    (setq bbam *tablecloth-bba-msg*)
    (print-ros-msg bbam)
    (unless bbam (return-from move-to-tablecloth-bb-front-old nil))
    (setq bb (car (send bbam :boxes)))
    (setq cube (make-cube (elt (ros::tf-point->pos (send bb :dimensions)) 0)
                          (elt (ros::tf-point->pos (send bb :dimensions)) 1)
                          (elt (ros::tf-point->pos (send bb :dimensions)) 2)
                          :pos (send (ros::tf-pose->coords (send bb :pose)) :pos)
                          :rot (send (ros::tf-pose->coords (send bb :pose)) :rot)))
    (setq standing-coords (send (make-coords) :move-to cube :world))
    (send standing-coords :translate (float-vector 1000 0 0) :local)
    (setq standing-coords (send (send standing-coords :copy-worldcoords) :rotate (deg2rad 90) :x))
    (setq tmp-coords standing-coords)
    (setq standing-coords (send (send (make-coords) :move-to tmp-coords) :orient (elt (send (make-coords) :difference-rotation tmp-coords) 2) :z :world))
    (send standing-coords :rotate (deg2rad 180) :z)
    (send standing-coords :translate (float-vector 0 0 (- (elt (send standing-coords :pos) 2))) :world)
    (pub-posearray standing-coords)
    (let (map-to-foot)
      (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
        (ros::ros-error "wait-for-transform failed. Return from move-to-tablecloth-bb-front-old nil")
        (return-from move-to-tablecloth-bb-front-old nil)
        )
      (setq map-to-foot (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
      (when map-to-foot
        (setq standing-coords (send (send standing-coords :copy-worldcoords) :transform map-to-foot :world))
        ))
    (ros::ros-info "standing-coords:~A" standing-coords)
    (when move
      (send *ri* :move-to standing-coords :frame-id "/base_footprint"))
    ;; for debug
    ;; (send *pr2* :move-to (make-coords) :world)
    ;; (send *pr2* :move-to standing-coords)
    ;; (objects (list *pr2* (make-coords) standing-coords tmp-coords cube (ros::tf-pose->coords (send bb :pose))))
    ))

(setq *tablecloth-bb-fixed* nil)
(defun sub-tablecloth-bb-fixed (msg)
  ;; (ros::ros-info "sub-tablecloth-bb-fixed cb")
  (setq *tablecloth-bb-fixed* msg)
  )

(setq *stay-back-distance* -600)
(defun adjust-tablecloth-bb-front-position (move)
  (let (bbam
        bb
        standing-coords
        posestamped
        bb-fixed-front-face
        z-axis-bb-rotaion
        )
    (look-at-tablecloth-bb)
    (ros::spin-once)
    (setq bbam *tablecloth-bb-fixed*)
    (when bbam (setq bb (car (send bbam :boxes))))
    (unless bb
      (ros::ros-error "Can't see enough region of bounding box.")
      (return-from adjust-tablecloth-bb-front-position nil))
    (setq posestamped (instance geometry_msgs::PoseStamped :init))
    (send posestamped :pose (send bb :pose))
    (send posestamped :header (send bbam :header))
    (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
      (ros::ros-error "wait-for-transform failed. Return from adjust-tablecloth-bb-front-position nil")
      (return-from adjust-tablecloth-bb-front-position nil)
      )
    (setq bb-fixed-front-face (send *tfl* :transform-pose "/base_footprint" posestamped))
    (send bb-fixed-front-face :translate
          (float-vector (/ (send bb :dimensions :x) 2) 0 0) :local)
    (setq z-axis-bb-rotation
          (elt (send bb-fixed-front-face :difference-rotation
                        (send (send (send (make-coords) :rotate (deg2rad -90) :y :world)
                                    :rotate (deg2rad -90) :x :world) :rotate (deg2rad 90) :z :world)) 1))
    (setq standing-coords (send bb-fixed-front-face :orient 0 :z :world))
    (send standing-coords :rotate z-axis-bb-rotation :z :world)
    (send standing-coords :translate
          (float-vector *stay-back-distance* 0 (- (elt (send standing-coords :pos) 2))) :local)
    (pub-posearray standing-coords "/adjust_standing_pos" "/base_footprint")
    (ros::ros-info "standing-coords:~A" standing-coords)
    (when move (send *ri* :go-pos-unsafe (/ (elt (send standing-coords :pos) 0) 1000) (/ (elt (send standing-coords :pos) 1) 1000) (rad2deg z-axis-bb-rotation)))
    ))

(setq *bottom-offset* 100)
(setq *move-distance-thre-x* 70)
(setq *move-distance-thre-y* 50)
(defun grab-bbside (&optional (move t))
  (let (bbam
        bb
        posestamped
        bb-fixed-front-face
        input-center-coords
        input-width
        )
    (send *pr2* :head :neck-p :joint-angle 50)
    (send *pr2* :head :neck-y :joint-angle 0) ;; default head position
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (look-at-tablecloth-bb)
    (unix:sleep 1)
    (ros::spin-once)
    (setq bbam *tablecloth-bb-fixed*)
    (when bbam (setq bb (car (send bbam :boxes))))
    (unless bb
      (ros::ros-error "Can't see enough region of bounding box.")
      (return-from grab-bbside nil))
    (setq posestamped (instance geometry_msgs::PoseStamped :init))
    (send posestamped :pose (send bb :pose))
    (send posestamped :header (send bbam :header))
    (unless (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 1)
      (ros::ros-error "wait-for-transform failed. Return from grab-bbside nil")
      (return-from grab-bbside nil)
      )
    (setq bb-fixed-front-face (send *tfl* :transform-pose "/base_footprint" posestamped))
    (send bb-fixed-front-face :translate
          (float-vector (/ (send bb :dimensions :x) 2) 0 0) :local)
    ;; (setq input-center-coords
    ;;       (send (send bb-fixed-front-face :copy-worldcoords) :translate
    ;;             (float-vector 0 (- (elt (send bb-fixed-front-face :pos) 1)) 0) :world))
    (setq input-center-coords (send bb-fixed-front-face :copy-worldcoords))
    (send input-center-coords :orient 0 :z :world)
    (send input-center-coords :translate
          (float-vector 0 0 (+ (* 1000 (- (/ (send bb :dimensions :y) 2))) *bottom-offset*)) :world)
    (setq input-width (* 1000 (send bb :dimensions :z)))
    (ros::ros-info "center-cds:~A width:~A" input-center-coords input-width)
    (ros::ros-info "move-pos:~A ~A"
                   (/ (- (elt (send input-center-coords :pos) 0) 500) 1000)
                   (/ (elt (send input-center-coords :pos) 1) 1000))
    (unless move (return-from grab-bbside nil))
    (when (or
           (> (abs (- (elt (send input-center-coords :pos) 0) 500)) *move-distance-thre-x*)
           (> (abs (elt (send input-center-coords :pos) 1)) *move-distance-thre-y*))
        (send *ri* :go-pos-unsafe
              (/ (- (elt (send input-center-coords :pos) 0) 500) 1000)
              (/ (elt (send input-center-coords :pos) 1) 1000)
              0)
        (grab-bbside)
        )
    ;; (grab-otherside input-center-coords input-width)
    ))

;; (defun rarm-grab-otherside (&optional grab-cds)
;;   (let (
;;         (grab-cds (cond (grab-cds grab-cds) (t *rarm-grab-cds*)))
;;         approach-coords
;;         )
;;     (send *pr2* :reset-pose)
;;     (send *ri* :stop-grasp :rarm)
;;     (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)
;;     (setq approach-coords
;;           (send (send (send (send grab-cds :copy-worldcoords)
;;                             :rotate (deg2rad 90) :x) :rotate (deg2rad 90) :y)
;;                 :rotate (deg2rad -90) :y :world))
;;     (send *pr2* :rarm :inverse-kinematics
;;           (send (send approach-coords :copy-worldcoords) :translate (float-vector 0 -100 0) :world)
;;           :rotation-axis t)
;;     (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)
;;     (send *pr2* :rarm :inverse-kinematics approach-coords :rotation-axis t)
;;     (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)
;;     ))

;; (defun larm-grab-otherside (&optional grab-cds)
;;   (let (
;;         (grab-cds (cond (grab-cds grab-cds) (t *larm-grab-cds*)))
;;         approach-coords
;;         )
;;     (send grab-cds :rotate (deg2rad -180) :x) ;; これ重要。これでほぼ左右対称になる
;;     (send *pr2* :reset-pose)
;;     (send *ri* :stop-grasp :larm)
;;     (send *ri* :larm-angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)
;;     (setq approach-coords
;;           (send (send (send (send (send grab-cds :copy-worldcoords)
;;                                   :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :y)
;;                       :rotate (deg2rad 180) :z)
;;                 :rotate (deg2rad -90) :y :world))
;;     (send *pr2* :larm :inverse-kinematics
;;           (send (send approach-coords :copy-worldcoords) :translate (float-vector 0 100 0) :world)
;;           :rotation-axis t)
;;     (send *ri* :larm-angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)
;;     (send *pr2* :larm :inverse-kinematics approach-coords :rotation-axis t)
;;     (send *ri* :larm-angle-vector (send *pr2* :angle-vector) 3000)
;;     (send *ri* :wait-interpolation)
;;     ))

(defun larm-grab-otherside (&key l-cds move)
  (let (
        (larm-grab-cds (cond (l-cds l-cds) (t *larm-grab-cds*)))
        larm-approach-coords
        res
        flag
        cnt
        ccnt
        direction-check
        set-pose
        grab-pose
        )
    (send larm-grab-cds :translate *larm-grab-offset* :world)
    (send larm-grab-cds :rotate (deg2rad -180) :x) ;; これ重要。これでほぼ左右対称になる
    (send *ri* :stop-grasp :larm)
    (ros::ros-info "larm-cds:~A" larm-grab-cds)
    (setq larm-approach-coords
          (send (send (send (send (send larm-grab-cds :copy-worldcoords)
                                  :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :y)
                      :rotate (deg2rad 180) :z)
                :rotate (deg2rad -90) :y :world))
    (setq flag nil)
    (setq cnt 0)
    (while (null (or flag (> cnt 10)))
      (setq larm-set-pose
            (send *pr2* :larm :inverse-kinematics
                  (send (send larm-approach-coords :copy-worldcoords)
                        :translate (float-vector 0 100 0) :world) :rotation-axis t))
      (setq set-pose (send *pr2* :angle-vector))
      (send *pr2* :reset-pose)
      (setq larm-grab-pose
            (send *pr2* :larm :inverse-kinematics larm-approach-coords :rotation-axis t))
      (setq grab-pose (send *pr2* :angle-vector))
      (setq res (list larm-set-pose larm-grab-pose))
      (ros::ros-warn "res:~A" res)
      (cond
       ((> (count nil res) 0)
        ;; can't solve ik. translate coords to upper
        (send larm-approach-coords :translate (float-vector 0 0 50) :world)
        (incf cnt)
        )
       (t ;; solved ik
        (ros::ros-warn "cnt:~A" cnt)
        (setq flag t)
        )))
    (when move
      (send *ri* :angle-vector
            (set-potentio-vector set-pose :head :torso :rarm) 5000)
      (send *ri* :wait-interpolation)
      (unix:sleep 2)
      (send *ri* :angle-vector
            (set-potentio-vector grab-pose :head :torso :rarm) 3000)
      (send *ri* :wait-interpolation)
      )
    (ros::ros-info "larm-approach-cds:~A" larm-approach-coords)
    (setq flag nil)
    (setq ccnt 0)
    (when (> cnt 0)
      ;; move arms as low as possible
      (while (null (or (> ccnt (* 10 cnt)) flag))
        (send larm-approach-coords :translate (float-vector 0 0 -5) :world)
        (send *pr2* :reset-pose)
        (setq larm-grab-pose
              (send *pr2* :larm :inverse-kinematics larm-approach-coords :rotation-axis t))
        (ros::ros-warn "count:~A" (count nil (list larm-grab-pose)))
        (when (= (count nil (list larm-grab-pose)) 1)
          ;; break
          (setq flag t)
          )
        (ros::ros-info "larm-approach-cds:~A" larm-approach-coords)
        (ros::ros-info "cnt:~A ccnt:~A" cnt ccnt)
        (incf ccnt)
        (when move
          (when larm-grab-pose
            (send *ri* :larm-angle-vector
                  (set-potentio-vector (send *pr2* :angle-vector) :head :torso) 2000)
          ))))
    (when move
      (send *ri* :wait-interpolation)
      (send *ri* :start-grasp :larm))
    (ros::ros-info "cnt:~A ccnt:~A" cnt ccnt)
    (ros::ros-info "larm-approach-cds:~A" larm-approach-coords)
    (return-from larm-grab-otherside t)
    ))


(defun mirror-arms (from angle-vector)
  (let(a v)
    (unless from (return-from mirror-arms nil))
    (unless angle-vector (return-from mirror-arms nil))
    (setq a (copy-object angle-vector))
    (setq v (copy-object angle-vector))
    (case from
      (:larm
       (setf (elt v 1) (- (elt a 8)))
       (setf (elt v 2) (elt a 9))
       (setf (elt v 3) (- (elt a 10)))
       (setf (elt v 4) (elt a 11))
       (setf (elt v 5) (- (elt a 12)))
       (setf (elt v 6) (elt a 13))
       (setf (elt v 7) (elt a 14))
       )
      (:rarm
       (setf (elt v 8) (- (elt a 1)))
       (setf (elt v 9) (elt a 2))
       (setf (elt v 10) (- (elt a 3)))
       (setf (elt v 11) (elt a 4))
       (setf (elt v 12) (- (elt a 5)))
       (setf (elt v 13) (elt a 6))
       (setf (elt v 14) (elt a 7))
       )
      (:arms
       (setf (elt v 1) (- (elt a 8)))
       (setf (elt v 2) (elt a 9))
       (setf (elt v 3) (- (elt a 10)))
       (setf (elt v 4) (elt a 11))
       (setf (elt v 5) (- (elt a 12)))
       (setf (elt v 6) (elt a 13))
       (setf (elt v 7) (elt a 14))
       (setf (elt v 8) (- (elt a 1)))
       (setf (elt v 9) (elt a 2))
       (setf (elt v 10) (- (elt a 3)))
       (setf (elt v 11) (elt a 4))
       (setf (elt v 12) (- (elt a 5)))
       (setf (elt v 13) (elt a 6))
       (setf (elt v 14) (elt a 7))
       ))
    v))

(defun mirror-ik (target-coords &rest args &key (reset t) rotation-axis r-thre)
  ;;(mirror-ik cds :rarm :rotation-axis t)
  ;; mirror cds and solve ik and mirror arms
  (let (tmp-cds res)
    (setq tmp-cds (send target-coords :copy-worldcoords))
    (send tmp-cds :translate (float-vector 0 (* 2 (- (elt (send tmp-cds :worldcoords) 1))) 0))
    (when reset
      (send *pr2* :reset-pose))
    (case (car args)
      (:rarm
       (setq res (send *pr2* :larm :inverse-kinematics tmp-cds
                       :rotation-axis rotation-axis :r-thre r-thre))
       (mirror-arms :larm res))
      (:larm
       (setq res (send *pr2* :rarm :inverse-kinematics tmp-cds
                       :rotation-axis rotation-axis :r-thre r-thre))
       (mirror-arms :rarm res)))))

(setq *larm-grab-offset* (float-vector 0 -40 60))
(setq *rarm-grab-offset* (float-vector 0 40 60))
(setq *rarm-grab-cds* (send (make-coords) :locate (float-vector 530 -350 600)))
(setq *larm-grab-cds* (send (make-coords) :locate (float-vector 530 350 600)))

(setq *rarm-grab-cds* (send (make-coords) :locate (float-vector 530 -400 200)))
(setq *larm-grab-cds* (send (make-coords) :locate (float-vector 530 400 200)))

;; (defun arms-grab-otherside-old (&key l-cds r-cds move)
;;   (let (
;;         (larm-grab-cds (cond (l-cds l-cds) (t *larm-grab-cds*)))
;;         (rarm-grab-cds (cond (r-cds r-cds) (t *rarm-grab-cds*)))
;;         larm-approach-coords
;;         rarm-approach-coords
;;         res
;;         flag
;;         cnt
;;         ccnt
;;         direction-check
;;         set-pose
;;         grab-pose
;;         )
;;     (send larm-grab-cds :translate *larm-grab-offset* :world)
;;     (send rarm-grab-cds :translate *rarm-grab-offset* :world)
;;     (send larm-grab-cds :rotate (deg2rad -180) :x) ;; これ重要。これでほぼ左右対称になる
;;     (send *ri* :stop-grasp)
;;     ;; (when move
;;     ;;   (send *ri* :angle-vector (set-potentio-vector (send *pr2* :reset-pose) :head :torso) 3000)
;;     ;;   (send *ri* :wait-interpolation))
;;     (ros::ros-info "larm-cds:~A rarm-cds:~A" larm-grab-cds rarm-grab-cds)
;;     (setq larm-approach-coords
;;           (send (send (send (send (send larm-grab-cds :copy-worldcoords)
;;                                   :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :y)
;;                       :rotate (deg2rad 180) :z)
;;                 :rotate (deg2rad -90) :y :world))
;;     (setq rarm-approach-coords
;;           (send (send (send (send rarm-grab-cds :copy-worldcoords)
;;                             :rotate (deg2rad 90) :x) :rotate (deg2rad 90) :y)
;;                 :rotate (deg2rad -90) :y :world))
;;     (setq flag nil)
;;     (setq cnt 0)
;;     (while (null (or flag (> cnt 10)))
;;       ;; (while (or (null flag) (> cnt 10))
;;       (setq larm-set-pose
;;             (send *pr2* :larm :inverse-kinematics
;;                   (send (send larm-approach-coords :copy-worldcoords)
;;                         :translate (float-vector 0 100 0) :world) :rotation-axis t))
;;       (setq rarm-set-pose
;;             (send *pr2* :rarm :inverse-kinematics
;;                   (send (send rarm-approach-coords :copy-worldcoords)
;;                         :translate (float-vector 0 -100 0) :world) :rotation-axis t))
;;       (setq set-pose (send *pr2* :angle-vector))
;;       (send *pr2* :reset-pose)
;;       (setq larm-grab-pose
;;             (send *pr2* :larm :inverse-kinematics larm-approach-coords :rotation-axis t))
;;       (setq rarm-grab-pose
;;             (send *pr2* :rarm :inverse-kinematics rarm-approach-coords :rotation-axis t))
;;       (setq grab-pose (send *pr2* :angle-vector))
;;       (setq res (list larm-set-pose rarm-set-pose larm-grab-pose rarm-grab-pose))
;;       ;; check whether solved ik is valid
;;       ;; (setq rarm-v (v-
;;       ;;               (send (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords)
;;       ;;                           :translate (float-vector 1 0 0) :local) :pos)
;;       ;;               (send (send *pr2* :rarm :end-coords) :worldpos)))
;;       ;; (setq larm-v (v-
;;       ;;               (send (send (send (send *pr2* :larm :end-coords) :copy-worldcoords)
;;       ;;                           :translate (float-vector 1 0 0) :local) :pos)
;;       ;;               (send (send *pr2* :larm :end-coords) :worldpos)))
;;       ;; (setq direction-check (and (> (elt rarm-v 1) 0) (< (elt larm-v 1) 0)))
;;       ;; (ros::ros-info "cnt:~A direction-check:~A ik-nil-cnt:~A" cnt direction-check (count nil res))
;;       (ros::ros-warn "res:~A" res)
;;       (cond
;;        ((> (count nil res) 0)
;;         ;; can't solve ik. translate coords to upper
;;         (send larm-approach-coords :translate (float-vector 0 0 50) :world)
;;         (send rarm-approach-coords :translate (float-vector 0 0 50) :world)
;;         (incf cnt)
;;         )
;;        (t ;; solved ik
;;         (ros::ros-warn "cnt:~A" cnt)
;;         (setq flag t)
;;         )))
;;     (when move
;;       (send *ri* :angle-vector
;;             (set-potentio-vector set-pose :head :torso) 5000)
;;       (send *ri* :wait-interpolation)
;;       (unix:sleep 2)
;;       (send *ri* :angle-vector
;;             (set-potentio-vector grab-pose :head :torso) 3000)
;;       (send *ri* :wait-interpolation)
;;       )
;;     (ros::ros-info "larm-approach-cds:~A rarm-approach-cds:~A"
;;                    larm-approach-coords rarm-approach-coords)
;;     (setq flag nil)
;;     (setq ccnt 0)
;;     (when (> cnt 0)
;;       ;; move arms as low as possible
;;       (while (null (or (> ccnt (* 10 cnt)) flag))
;;         ;; (while (or (< ccnt (* 10 cnt)) (null flag))
;;         (send larm-approach-coords :translate (float-vector 0 0 -5) :world)
;;         (send rarm-approach-coords :translate (float-vector 0 0 -5) :world)
;;         (send *pr2* :reset-pose)
;;         (setq larm-grab-pose
;;               (send *pr2* :larm :inverse-kinematics larm-approach-coords :rotation-axis t))
;;         (setq rarm-grab-pose
;;               (send *pr2* :rarm :inverse-kinematics rarm-approach-coords :rotation-axis t))
;;         (ros::ros-warn "count:~A" (count nil (list larm-grab-pose rarm-grab-pose)))
;;         (when (= (count nil (list larm-grab-pose rarm-grab-pose)) 2)
;;           ;; break
;;           (setq flag t)
;;           )
;;         (ros::ros-info "larm-approach-cds:~A rarm-approach-cds:~A"
;;                        larm-approach-coords rarm-approach-coords)
;;         (ros::ros-info "cnt:~A ccnt:~A" cnt ccnt)
;;         (incf ccnt)
;;         (when move
;;           (when larm-grab-pose
;;             (send *ri* :larm-angle-vector
;;                   (set-potentio-vector (send *pr2* :angle-vector) :head :torso) 2000)
;;             )
;;           (when rarm-grab-pose
;;             (send *ri* :rarm-angle-vector
;;                   (set-potentio-vector (send *pr2* :angle-vector) :head :torso) 2000)
;;             ))))
;;     (when move
;;       (send *ri* :wait-interpolation)
;;       (send *ri* :start-grasp))
;;     (ros::ros-info "cnt:~A ccnt:~A" cnt ccnt)
;;     (ros::ros-info "larm-approach-cds:~A rarm-approach-cds:~A"
;;                    larm-approach-coords rarm-approach-coords)
;;     (return-from arms-grab-otherside t)
;;     ))

(defun grab-otherside (&key (use-arm :arms) l-cds r-cds move)
  (let (
        (larm-grab-cds (send (cond (l-cds l-cds) (t *larm-grab-cds*)) :copy-worldcoords))
        (rarm-grab-cds (send (cond (r-cds r-cds) (t *rarm-grab-cds*)) :copy-worldcoords))
        larm-approach-coords
        rarm-approach-coords
        larm-grab-pose
        rarm-grab-pose
        res
        flag
        cnt
        debug-count
        ccnt
        direction-check
        set-pose
        grab-pose
        use-arm-lst
        )
    (case use-arm
      (:rarm
       (setq use-arm-lst (list :rarm)))
      (:larm
       (setq use-arm-lst (list :larm)))
      ((:arms)
       (setq use-arm-lst (list :rarm :larm))))
    (send larm-grab-cds :translate *larm-grab-offset* :world)
    (send rarm-grab-cds :translate *rarm-grab-offset* :world)
    (send larm-grab-cds :rotate (deg2rad -180) :x) ;; これ重要。これでほぼ左右対称になる
    (cond
     ((eq use-arm :arms)
      (send *ri* :stop-grasp))
     (t
      (send *ri* :stop-grasp use-arm)))
    (ros::ros-info "larm-cds:~A rarm-cds:~A" larm-grab-cds rarm-grab-cds)
    (setq larm-approach-coords
          (send (send (send (send (send larm-grab-cds :copy-worldcoords)
                                  :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :y)
                      :rotate (deg2rad 180) :z)
                :rotate (deg2rad -90) :y :world))
    (setq rarm-approach-coords
          (send (send (send (send rarm-grab-cds :copy-worldcoords)
                            :rotate (deg2rad 90) :x) :rotate (deg2rad 90) :y)
                :rotate (deg2rad -90) :y :world))
    (setq flag nil)
    (setq cnt 0)
    (while (null (or flag (> cnt 10)))
      (when (member :larm use-arm-lst)
        (setq larm-set-pose
              (send *pr2* :larm :inverse-kinematics
                    (send (send larm-approach-coords :copy-worldcoords)
                          :translate (float-vector 0 100 0) :world) :rotation-axis t))
        )
      (when (member :rarm use-arm-lst)
        (setq rarm-set-pose
              (send *pr2* :rarm :inverse-kinematics
                    (send (send rarm-approach-coords :copy-worldcoords)
                          :translate (float-vector 0 -100 0) :world) :rotation-axis t))
        )
      (setq set-pose (send *pr2* :angle-vector))
      (send *pr2* :reset-pose)
      (when (member :larm use-arm-lst)
        (setq larm-grab-pose
              (send *pr2* :larm :inverse-kinematics larm-approach-coords :rotation-axis t))
        )
      (when (member :rarm use-arm-lst)
        (setq rarm-grab-pose
              (send *pr2* :rarm :inverse-kinematics rarm-approach-coords :rotation-axis t))
        )
      (setq grab-pose (send *pr2* :angle-vector))
      (case use-arm
        (:larm
         (setq res (list larm-set-pose larm-grab-pose))
         )
        (:rarm
         (setq res (list rarm-set-pose rarm-grab-pose))
         )
        (:arms
         (setq res (list larm-set-pose rarm-set-pose larm-grab-pose rarm-grab-pose))
         ))
      (ros::ros-warn "res:~A" res)
      (cond
       ((> (count nil res) 0)
        (send larm-approach-coords :translate (float-vector 0 0 50) :world)
        (send rarm-approach-coords :translate (float-vector 0 0 50) :world)
        (incf cnt)
        )
       (t ;; solved ik
        (ros::ros-warn "cnt:~A" cnt)
        (setq flag t)
        )))
    (when move
      (case use-arm
        (:larm
         (send *ri* :angle-vector
               (set-potentio-vector set-pose :head :torso :rarm) 5000)
         (send *ri* :wait-interpolation)
         (unix:sleep 2)
         (send *ri* :angle-vector
               (set-potentio-vector grab-pose :head :torso :rarm) 3000)
         (send *ri* :wait-interpolation)
         )
        (:rarm
         (send *ri* :angle-vector
               (set-potentio-vector set-pose :head :torso :larm) 5000)
         (send *ri* :wait-interpolation)
         (unix:sleep 2)
         (send *ri* :angle-vector
               (set-potentio-vector grab-pose :head :torso :larm) 3000)
         (send *ri* :wait-interpolation)
        )
        (:arms
         (send *ri* :angle-vector
               (set-potentio-vector set-pose :head :torso) 5000)
         (send *ri* :wait-interpolation)
         (unix:sleep 2)
         (send *ri* :angle-vector
               (set-potentio-vector grab-pose :head :torso) 3000)
         (send *ri* :wait-interpolation)
         ))
      )
    (ros::ros-info "larm-approach-cds:~A rarm-approach-cds:~A"
                   larm-approach-coords rarm-approach-coords)
    (setq flag nil)
    (setq ccnt 0)
    (when (> cnt 0)
      ;; move arms as low as possible
      (while (null (or (> ccnt (* 10 cnt)) flag))
        (send larm-approach-coords :translate (float-vector 0 0 -5) :world)
        (send rarm-approach-coords :translate (float-vector 0 0 -5) :world)
        (send *pr2* :reset-pose)
        (when (member :larm use-arm-lst)
          (setq larm-grab-pose
                (send *pr2* :larm :inverse-kinematics larm-approach-coords :rotation-axis t))
          )
        (when (member :rarm use-arm-lst)
          (setq rarm-grab-pose
                (send *pr2* :rarm :inverse-kinematics rarm-approach-coords :rotation-axis t))
          )
        (case use-arm
          (:larm
           (setq debug-count (count nil (list larm-grab-pose)))
           )
          (:rarm
           (setq debug-count (count nil (list rarm-grab-pose)))
           )
          (:arms
           (setq debug-count (count nil (list larm-grab-pose rarm-grab-pose)))
           ))
        (ros::ros-warn "count:~A" debug-count)
        (when (= debug-count (length use-arm-lst))
          ;; break
          (setq flag t)
          )
        (ros::ros-info "larm-approach-cds:~A rarm-approach-cds:~A"
                       larm-approach-coords rarm-approach-coords)
        (ros::ros-info "cnt:~A ccnt:~A" cnt ccnt)
        (incf ccnt)
        (when move
          (when larm-grab-pose
            (send *ri* :larm-angle-vector
                  (set-potentio-vector (send *pr2* :angle-vector) :head :torso) 2000)
            )
          (when rarm-grab-pose
            (send *ri* :rarm-angle-vector
                  (set-potentio-vector (send *pr2* :angle-vector) :head :torso) 2000)
            ))))
    (when move
      (send *ri* :wait-interpolation)
      (case use-arm
        (:larm
         (send *ri* :start-grasp :larm))
        (:rarm
         (send *ri* :start-grasp :rarm))
        (:arms
         (send *ri* :start-grasp)
         )))
    (ros::ros-info "cnt:~A ccnt:~A" cnt ccnt)
    (ros::ros-info "larm-approach-cds:~A rarm-approach-cds:~A"
                   larm-approach-coords rarm-approach-coords)
    (return-from grab-otherside t)
    ))

(setq *downleft-posearray* nil)
(defun sub-downleft (msg)
  ;; (ros::ros-info "sub-downleft cb")
  (setq *downleft-posearray* msg))
(setq *downright-posearray* nil)
(defun sub-downright (msg)
  ;; (ros::ros-info "sub-downright cb")
  (setq *downright-posearray* msg))

(defun set-potentio-vector (vl &rest controllers)
  (let(
       tmp-vector
       potentio-vector
       )
    (cond
     ((listp vl)
      (setq tmp-vector (copy-object vl)))
     (t
      (setq tmp-vector (list (copy-object vl)))))
    (setq potentio-vector (send *ri* :state :potentio-vector))
    (dolist (v tmp-vector)
      (dolist (con controllers)
        (case con
          (:torso
           (setf (elt v 0) (elt potentio-vector 0)))
          (:larm
           (setf (elt v 1) (elt potentio-vector 1))
           (setf (elt v 2) (elt potentio-vector 2))
           (setf (elt v 3) (elt potentio-vector 3))
           (setf (elt v 4) (elt potentio-vector 4))
           (setf (elt v 5) (elt potentio-vector 5))
           (setf (elt v 6) (elt potentio-vector 6))
           (setf (elt v 7) (elt potentio-vector 7)))
          (:rarm
           (setf (elt v 8) (elt potentio-vector 8))
           (setf (elt v 9) (elt potentio-vector 9))
           (setf (elt v 10) (elt potentio-vector 10))
           (setf (elt v 11) (elt potentio-vector 11))
           (setf (elt v 12) (elt potentio-vector 12))
           (setf (elt v 13) (elt potentio-vector 13))
           (setf (elt v 14) (elt potentio-vector 14)))
          (:head
           (setf (elt v 15) (elt potentio-vector 15))
           (setf (elt v 16) (elt potentio-vector 16)))
          )))
    (cond
     ((listp vl)
      tmp-vector)
     (t
      (car tmp-vector)))))

(setq *neck-p-max* 60)
(setq *downleft-lowest* nil)
(setq *downright-lowest* nil)
(setq *lowest-center* nil)
(setq *lowest-thre* 100) ;; cut ground
(defun search-lowest-point (&optional neck-p-max)
  (let
      (l-cds r-cds deg deg-max)
    (ros::spin-once)
    (setq l-cds *downleft-posearray*)
    (setq r-cds *downright-posearray*)
    (send *pr2* :head :neck-p :joint-angle 0)
    (send *ri* :angle-vector (set-potentio-vector (send *pr2* :angle-vector) :torso :rarm :larm) 3000)
    (send *ri* :wait-interpolation)
    ;; again for avoiding ground point loop
    (ros::spin-once)
    (setq l-cds *downleft-posearray*)
    (setq r-cds *downright-posearray*)
    (cond
     (neck-p-max
      (setq deg-max neck-p-max)
      )
     (t
      (setq deg-max *neck-p-max*)
      ))
    (dotimes (i 90)
      (setq deg (* deg-max (sin (deg2rad i))))
      (send *pr2* :head :neck-p :joint-angle deg)
      (send *ri* :angle-vector (set-potentio-vector (send *pr2* :angle-vector) :torso :rarm :larm) 1000)
      ;; (send *ri* :wait-interpolation)
      (unix:usleep 100000)
      (ros::spin-once)
      (when *downleft-posearray*
        (ros::ros-warn
         "downleft:~A"
         (send (ros::tf-pose->coords (car (send *downleft-posearray* :poses))) :worldpos))
        (ros::ros-warn "l-cds:~A"
                       (send (ros::tf-pose->coords (car (send l-cds :poses))) :worldpos))
        (when
            (and
             (> (elt (send (ros::tf-pose->coords (car (send *downleft-posearray* :poses))) :worldpos) 2) *lowest-thre*)
             (>
              (elt (send (ros::tf-pose->coords (car (send l-cds :poses))) :worldpos) 2)
              (elt (send (ros::tf-pose->coords (car (send *downleft-posearray* :poses))) :worldpos) 2)
              ));; z is lower
          (ros::ros-warn "Found downleft lower point. Now(~A) > New(~A)"
                         (elt (send (ros::tf-pose->coords (car (send l-cds :poses))) :worldpos) 2)
                         (elt (send (ros::tf-pose->coords (car (send *downleft-posearray* :poses))) :worldpos) 2)
                         )
          (setq l-cds *downleft-posearray*)
          ))
      (when *downright-posearray*
        (ros::ros-warn
         "downright:~A"
         (send (ros::tf-pose->coords (car (send *downright-posearray* :poses))) :worldpos))
        (ros::ros-warn
         "r-cds:~A" (send (ros::tf-pose->coords (car (send r-cds :poses))) :worldpos))
        (when
            (and
             (> (elt (send (ros::tf-pose->coords (car (send *downright-posearray* :poses))) :worldpos) 2) *lowest-thre*)
             (>
              (elt (send (ros::tf-pose->coords (car (send r-cds :poses))) :worldpos) 2)
              (elt (send (ros::tf-pose->coords (car (send *downright-posearray* :poses))) :worldpos) 2)
              ));; z is lower
          (ros::ros-warn "Found downright lower point. Now(~A) > New(~A)"
                         (elt (send (ros::tf-pose->coords (car (send r-cds :poses))) :worldpos) 2)
                         (elt (send (ros::tf-pose->coords (car (send *downright-posearray* :poses))) :worldpos) 2)
                         )
          (setq r-cds *downright-posearray*)
          )))
    (when (or (null l-cds) (null r-cds))
      (ros::ros-error "Couldn't find lowest point.")
      (return-from search-lowest-point nil)
      )
    (ros::ros-warn "Downleft  lowest point: ~A"
                   (send (ros::tf-pose->coords (car (send r-cds :poses))) :worldpos))
    (ros::ros-warn "Downright lowest point: ~A"
                   (send (ros::tf-pose->coords (car (send l-cds :poses))) :worldpos))
    (setq *lowest-center*
          (scale 0.5 (v+
                      (send (ros::tf-pose->coords (car (send r-cds :poses))) :worldpos)
                      (send (ros::tf-pose->coords (car (send l-cds :poses))) :worldpos))))
    (send *pr2* :head :look-at *lowest-center*)
    (send *ri* :angle-vector (set-potentio-vector (send *pr2* :angle-vector)
                                                  :torso :rarm :larm))
    (send *ri* :wait-interpolation)
    (setq *downleft-lowest* l-cds)
    (setq *downright-lowest* r-cds)
  ))

(defun point-based-grab-otherside (&key (move t))
  (let
      (tuckarm-to-resetpose-lst downleft-posearray downright-posearray open-pose)
    (setq tuckarm-to-resetpose-lst
          (set-potentio-vector
           (list
            (float-vector 11.7313 32.9328 61.3994 79.0585 -121.542 19.5933 -90.0975 -20.0071 -5.07364 69.7435 -105.023 -90.0214 69.9916 -5.72958 19.823 -19.6913 10.578)
            (float-vector 11.7313 57.7383 60.9632 72.2413 -121.542 23.2623 -90.2097 -20.0196 -5.09739 69.7386 -105.023 -90.0214 69.9916 -5.72958 19.823 -19.6913 10.578)
            (float-vector 11.7313 57.6576 60.8614 72.2321 -121.542 23.2822 -90.2122 -20.022 -14.308 62.4149 -101.88 -95.7282 69.9784 -5.72958 19.7582 -19.6913 10.578)
            (float-vector 11.7313 61.016 60.6094 71.2398 -121.542 23.9351 -91.0349 -20.0569 -41.0279 61.8866 -103.571 -105.217 69.9585 -5.72958 19.7258 -19.6913 10.542)
            )
           :head
           ))
    (setq open-pose (float-vector 11.5 91.6907 57.821 5.31744 -116.017 -0.002117 -5.78587 -0.000739 -90.9659 60.4606 3.90537 -117.187 0.000504 -5.72958 0.095981 -1.53988 53.166))
    (when move
      (send *ri* :stop-grasp)
      (send *ri* :wait-interpolation)
      (when (check-tuckarm-pose)
        (send *ri* :angle-vector-sequence tuckarm-to-resetpose-lst (list 1000 1000 1000 1000))
        (send *ri* :wait-interpolation)
        )
      (send *ri* :angle-vector (set-potentio-vector open-pose :head :rarm))
      (send *ri* :wait-interpolation)
      (send *ri* :angle-vector (set-potentio-vector open-pose :head))
      (send *ri* :wait-interpolation)
      )
    (unix:sleep 2)
    (ros::spin-once) ;; get data
    (cond
     (*downleft-posearray*
      (setq downleft-posearray *downleft-posearray*))
     (t
      (ros::ros-error "Couldn't get downleft-posearray. Will use default pose.")))
    (cond
     (*downright-posearray*
      (setq downright-posearray *downright-posearray*))
     (t
      (ros::ros-error "Couldn't get downright-posearray. Will use default pose.")))
    (cond
     ((check-both-arms-possible)
      (search-lowest-point)
      (grab-otherside
       :l-cds (cond (downleft-posearray
                     (ros::tf-pose->coords (car (send *downleft-lowest* :poses))))
                    (t nil))
       :r-cds (cond (downright-posearray
                     (ros::tf-pose->coords (car (send *downright-lowest* :poses))))
                    (t nil))
       :move move)
      )
     (t ;; grab one by one
      ;; (go-pos-unsafe)
      (search-lowest-point)
      (grab-otherside
       :use-arm :larm
       :l-cds (cond (downleft-posearray
                     (ros::tf-pose->coords (car (send *downleft-lowest* :poses))))
                    (t nil))
       :move move)
      ;; go-pos-unsafe
      (search-lowest-point)
      (grab-otherside
       :use-arm :rarm
       :r-cds (cond (downright-posearray
                     (ros::tf-pose->coords (car (send *downright-lowest* :poses))))
                    (t nil))
       :move move)
      ))
    (unless (check-grab-otherside t)
      (point-based-grab-otherside :move move))
    ))

(setq *both-arms-distance-thre* 0.85)
(defun check-both-arms-possible ()
  (ros::spin-once)
  (unless *raw-tablecloth-bb*
    (ros::ros-error "Couldn't get raw tablecloth boudningbox.")
    (check-both-arms-possible)
    )
  (when (< (send (car (send *raw-tablecloth-bb* :boxes)) :dimensions :z) *both-arms-distance-thre*)
    (ros::ros-info "Both arms possible!")
    (return-from check-both-arms-possible t))
  (return-from check-both-arms-possible nil)
  )

(setq *raw-tablecloth-bb* nil)
(defun sub-raw-tablecloth-bb (msg)
  (setq *raw-tablecloth-bb* msg))

(defun check-grab-otherside (&optional (init-pos nil))
  (let ((error-norm 0)
        (ev (float-vector 0 0 0 0 0 0 0 0 0 0 0 0 0 0))
        pr2-initial-pose
        l-coords
        r-coords
        l-init-cds
        r-init-cds
        cnt
        )
    (setq error-norm 0)
    (setq ev (float-vector 0 0 0 0 0 0 0 0 0 0 0 0 0 0))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq l-coords
          (send (send (send *pr2* :larm :end-coords) :copy-worldcoords)
                :translate (float-vector 0 0 200) :world))
    (setq r-coords
          (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords)
                :translate (float-vector 0 0 200) :world))
    (setq l-init-cds (copy-object l-coords))
    (setq r-init-cds (copy-object r-coords))
    (send *pr2* :rarm :inverse-kinematics r-coords :rotation-axis t)
    (send *pr2* :larm :inverse-kinematics l-coords :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (setq cnt 0)
    (while (< error-norm 0.3)
      (let ((error-vector (send *ri* :error-vector))
            )
        (dotimes (x 14)
          (setf (elt ev x) (elt error-vector (+ 1 x))))
        (ros::ros-info "error-vector:~A" error-vector)
        (ros::ros-info "ev:~A" ev)
        (ros::ros-info "norm:~A" (norm ev))
        (setq error-norm (norm ev))
        (setq res (list (send *pr2* :rarm :inverse-kinematics
                              (send r-coords :translate (float-vector 0 -15 0) :world)
                              :rotation-axis t)
                        (send *pr2* :larm :inverse-kinematics
                              (send l-coords :translate (float-vector 0 15 0) :world)
                              :rotation-axis t)))
        (when (= (count nil res) 0)
          (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
          (unix:usleep 300000)
          ;; (send *ri* :wait-interpolation)
          )
        (incf cnt)
        (when (eq cnt 10)
          (ros::ros-error "Grasp otherside failed.")
          (send *ri* :stop-grasp)
          (send *ri* :wait-interpolation)
          (return-from check-grab-otherside nil)
        )))
    (ros::ros-info "Grasp otherside finished.")
    (setq res (list (send *pr2* :rarm :inverse-kinematics r-init-cds :rotation-axis t)
                    (send *pr2* :larm :inverse-kinematics l-init-cds :rotation-axis t)))
    (ros::ros-info "init l:~A r:~A coords l:~A r:~A"
                   l-init-cds r-init-cds l-coords r-coords)
    (when (= (count nil res) 0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      )
    ;; (when init-pos (set-arms-to-movement-pos))
    (return-from check-grab-otherside t)
    ))

(defun set-arms-to-movement-pos (&key (torso t))
  (send *ri* :angle-vector
        (set-potentio-vector
         #f(299.899 50.8817 29.8494 83.8443 -111.679 10.9221 -18.7541 75.1476 -40.9226 55.003 -56.2095 -116.084 -79.3229 -46.6366 130.39 0.002118 -0.978)
         :head :torso))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
        (set-potentio-vector (send *pr2* :reset-pose) :rarm :larm
                             (cond (torso :torso) (t nil))))
  (send *ri* :wait-interpolation)
  )

;; (defun get-ave-cds ()
;;   (let (x-sum y-sum x y);; z = 0
;;     (unless (eq (length *table-cds-lst*) 10)
;;       ;; (ros::ros-error "Couldn't find enough data of table.")
;;       (return-from get-ave-cds nil))
;;     (setq x-sum 0)
;;     (setq y-sum 0)
;;     (dolist (i *table-cds-lst*)
;;       (setq x-sum (+ x-sum (elt i 0)))
;;       (setq y-sum (+ y-sum (elt i 1)))
;;       )
;;     (setq x (/ x-sum 10.0))
;;     (setq y (/ y-sum 10.0))
;;     (return-from get-ave-cds (float-vector x y 0))
;;     ))

;; (defun get-ave-rot ()
;;   (let (rot rot-sum);; z = 0
;;     (unless (eq (length *table-rot-lst*) 10)
;;       ;; (ros::ros-error "Couldn't find enough data of table.")
;;       (return-from get-ave-rot nil))
;;     (setq rot-sum 0)
;;     (dolist (i *table-rot-lst*)
;;       (setq rot-sum (+ rot-sum i))
;;       )
;;     (setq rot (/ rot-sum 10.0))
;;     (return-from get-ave-rot rot)
;;     ))

;; (setq *table-cds-lst* nil)
;; (setq *table-rot-lst* nil)
;; (setq *table-cds-thre* 100)
;; (defun sub-tablemarkerarray (msg)
;;   (let (tmp-coords ave-cds ave-rot ave-coords posearray)
;;     (setq tmp-coords (ros::tf-pose->coords (send (cadr (send msg :markers)) :pose)))
;;     ;; throw outlier
;;     (setq ave-cds (get-ave-cds))
;;     (setq ave-rot (get-ave-rot))
;;     (cond
;;      (ave-cds
;;       (ros::ros-info "norm:~A" (norm (v- (send tmp-coords :pos) ave-cds)))
;;       (when (< (norm (v- (send tmp-coords :pos) ave-cds)) *table-cds-thre*)
;;         (setq *table-cds-lst* (append *table-cds-lst* (list (send tmp-coords :pos))))
;;         (setq *table-rot-lst*
;;               (append *table-rot-lst*
;;                       (list (elt (send (make-coords) :difference-rotation tmp-coords) 2))))
;;       ))
;;      (t
;;       (setq *table-cds-lst* (append *table-cds-lst* (list (send tmp-coords :pos))))
;;       (setq *table-rot-lst*
;;             (append *table-rot-lst*
;;                     (list (elt (send (make-coords) :difference-rotation tmp-coords) 2))))
;;       ))
;;     (when t(> (length *table-cds-lst*) 10) ;; throw old data
;;       (setq *table-cds-lst* (cdr *table-cds-lst*))
;;       (setq *table-rot-lst* (cdr *table-rot-lst*))
;;       (setq ave-coords (send (make-coords) :translate ave-cds))
;;       (send ave-coords :rotate ave-rot :z :world)
;;       (setq posearray (instance geometry_msgs::PoseArray :init))
;;       (send posearray :header (send (car (send msg :markers)) :header))
;;       (send posearray :poses
;;             (list (instance geometry_msgs::Pose :init :pose (ros::coords->tf-pose ave-coords))))
;;       (ros::ros-info "pub")
;;       (ros::publish "/table_static" posearray)
;;       )))

(setq *table-static* nil)
(defun sub-tablestatic (msg)
  (setq *table-static* msg))

(defun demo ()
  (ros::spin-once)
  (unless *checkerboard-pos* (return-from demo nil))
  (approach)
  (grab)
  (spread-tablecloth)
  )

(defun app () (approach))
(defun spread () (spread-tablecloth))
(defun raise () (raise-arms))
(defun publish () (pub-tablecloth-bb))
(defun pub ()
  (while (ros::ok)
    (ros::spin-once)
    (publish)
    (Ros::sleep)))
(defun look () (look-at-tablecloth-bb))
(defun move () (move-to-tablecloth-bb-front))
(defun dumove () (move-to-tablecloth-bb-front :move nil))
(defun adjust () (adjust-tablecloth-bb-front-position t))
(defun dujust () (adjust-tablecloth-bb-front-position nil))
(defun fix () (grab-bbside))
(defun other (&optional (move t)) (point-based-grab-otherside :move move))
(defun search () (search-lowest-point))
(defun check () (check-grab-otherside t))
(defun reload () (load "demo.l"))
(defun measure () (measure-tablecloth-height))
(defun grabdemo ()
  (grab)
  (spread)
  (raise)
  )
(defun demo ()
  (move)
  (dujust)
  (fix)
  (other)
  )
(defun ph ()
  (warn ";; \(app\) \(approach\) checker boardを見て棚に近づく~%")
  (warn ";; \(grab\) grab-sideのpr2がclothを掴んで上に上げる~%")
  (warn ";; \(spread) \(spread-tablecloth\) clothを広げる~%")
  (warn ";; \(raise) \(raise-arms\) armsを上に上げる~%")
  (warn ";; \(publish\) \(pub-tablecloth-bb\) grab-sideのpr2がcloth相当のboudingboxをpub~%")
  (warn ";; \(pub\) \(publish\)し続ける~%")
  (warn ";; \(look\) \(look-at-tablecloth-bb\) grab-othersideのpr2がbbをlook~%")
  (warn ";; \(move\) \(move-to-tablecloth-bb-front\) grab-othersideのpr2がbb-frontへ移動~%")
  (warn ";; \(dumove\) \(move-to-tablecloth-bb-front nil\) dummy move~%")
  (warn ";; \(adjust\) \(adjust-tablecloth-bb-front-position t\) othersideがfixed_bbを元に微調整移動~%")
  (warn ";; \(dujust\) \(adjust-tablecloth-bb-front-position nil\) othersideがfixed_bbを元に微調整移動~%")
  (warn ";; \(fix\) \(grab-bbside\) othersideが最後の微調整移動~%")
  (warn ";; \(other\) \(point-based-grab-otherside\) othersideがothersideをgrab~%")
  (warn ";; \(search\) \(search-lowest-point\) othersideがsearch~%")
  (warn ";; \(check\) \(check-grab-otherside t\) othersideがothersideをgrabしてinit-posへ~%")
  (warn ";; \(reload\) \(load \"demo.l\"\)~%")
  (warn ";; \(measure\) \(measure-tablecloth-height\)~%")
  (warn ";; \(ph\) print help~%")
  (warn ";; \(demo\) otherside demo~%")
  (warn ";; \(grabdemo\) grabside demo~%")
  (warn ";; \(grab-bbside\) need fix~%")
  )
(ph)
