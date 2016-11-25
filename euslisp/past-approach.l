;; (load "package://jsk_perception/euslisp/detection_interface.l")
;; (ros::load-ros-manifest "jsk_smart_gui")
;; (require :jsk-smart-gui-utils "package://jsk_smart_gui/src/utils.l")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://jsk_demo_common/euslisp/pr2-move.l")

(defmethod pr2-interface
  (:start-grasp
   ;;;;;;wait t -> nilにしてみた
   (&optional (arm :arms) &key ((:gain g) 0.01) ((:objects objs) objects))
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait nil)
   (unless joint-action-enable
     (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
       (let ((grasp-convex
	      (convex-hull-3d
	       (flatten
		(mapcar
		 #'(lambda (l)
		     (send-all (send l :bodies) :worldcoords)
		     (send-all (send l :bodies) :vertices))
		 (send robot a :gripper :links))))))
	 (dolist (obj objs)
	   (when (and (find-method obj :faces)
		      (not (= (pqp-collision-check grasp-convex obj) 0)))
	     (if (send obj :parent) (send (send obj :parent) :dissoc obj))
	     (send robot a :end-coords :assoc obj)))))
     ;; (send self :update-robot-state) ;; update state of 'robot' for real robot
     (return-from :start-grasp
       (case arm
         (:larm (send robot :l_gripper_joint :joint-angle))
         (:rarm (send robot :r_gripper_joint :joint-angle))
         (t (list
             (send robot :l_gripper_joint :joint-angle)
             (send robot :r_gripper_joint :joint-angle))))))
   ;; for real robot
   (let ((clients (case arm
                    (:rarm (list (cons :r_gripper_joint r-gripper-action)))
                    (:larm (list (cons :r_gripper_joint l-gripper-action)))
                    (:arms (list (cons :r_gripper_joint r-gripper-action)
				 (cons :l_gripper_joint l-gripper-action)))
                    (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm))))
         aresult ajoint (resend-goal t) (resend-count 0) ret)
     (while (and resend-goal (< resend-count 2))
       (setq resend-goal nil)
       (send self :update-robot-state) ;; update state of 'robot' for real robot
       (dolist (client clients)
         (setq aresult (send (cdr client) :get-result)
               ajoint (send robot (car client)))
         (ros::ros-debug ";; :move-gripper check reached_goal ~A, position ~A, robot-body angle ~A" (send aresult :reached_goal) (* 1000 (send aresult :position)) (send ajoint :joint-angle))
         (unless (send aresult :reached_goal) ;; in case that reached_gal is fail, try one more time
           (ros::ros-debug ";; :move-gripper ~A need to resend goal, position = ~A, unreached" (send ajoint :name) (* 1000 (send aresult :position)))
           (setq resend-goal t))
       ;;
         (unless (eps= (* 1000 (send aresult :position)) (send ajoint :joint-angle) 2)
           (ros::ros-debug ";; :move-gripper ~A need to resend goal, position = ~A/~A, result and udpate-body differs" (send ajoint :name) (* 1000 (send aresult :position)) (send ajoint :joint-angle))
           (setq resend-goal t)))
       (incf resend-count)
       (if resend-goal (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait nil)))
     (setq ret
           (case arm
             (:rarm (* 1000 (send (send r-gripper-action :get-result) :position)))
             (:larm (* 1000 (send (send l-gripper-action :get-result) :position)))
             (:arms (mapcar #'(lambda (c) (* 1000 (send (send c :get-result) :position))) (list r-gripper-action l-gripper-action)))))
     (ros::ros-debug ";; :move-gripper ~A returns ~A" (send ajoint :name) ret)
     ret))
  (:get-grasp-result (arm)
   ;;;;start-graspの最後の部分だけ取ってきた
   nil
   (setq ret
         (case arm
           (:rarm (* 1000 (send (send r-gripper-action :get-result) :position)))
           (:larm (* 1000 (send (send l-gripper-action :get-result) :position)))
           (:arms (mapcar #'(lambda (c) (* 1000 (send (send c :get-result) :position))) (list r-gripper-action l-gripper-action)))))
   (ros::ros-debug ";; :move-gripper ~A returns ~A" (send ajoint :name) ret)
   ret)

  (:go-force
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (ros::ros-info "go-pos-unsafe (x y d) = (~A ~A ~A)" x y d)
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
           (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x 1000) (* y 1000) 0))
       (send coords :rotate (deg2rad d) :z)
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
           ((> curr-tm 1000))
         (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
         (if viewer (send self :draw-objects))))
     (return-from :go-pos-unsafe t))
   (let (msec step goal (maxvel 0.295) (maxrad 0.495) (counter 0))
     ;; package://pr2_base_trajectory_action/config/pr2_base_link.yaml
     ;; 80% of maxvel = 0.3[m/sec]
     ;; 80% of maxrad = 0.5[rad/sec]
     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
                             (/ (abs (deg2rad d)) (* maxrad 0.8)))))
     (setq msec (max msec 1000))
     (setq step (/ 1000.0 msec))
     (while (and (or (> (sqrt (+ (* x x) (* y y)))  0.025)
                     (> (abs d) 2.5) ;; 3 cm and 3deg
                     (= counter 0))  ;; try at least 1 time
                 (< counter 3)) ;; magic number 3 times
       (setq goal (send self :move-trajectory (* x step) (* y step)
                        (* (deg2rad d) step) msec :stop t))
       (send move-base-trajectory-action :send-goal goal)
       ;; (let ((acret
       ;;        (send move-base-trajectory-action :wait-for-result)))
       ;;   (unless acret
       ;;     (return-from :go-pos-unsafe nil))
       (send move-base-trajectory-action :spin-once)
       (send self :spin-once)
       (send self :spin-once)
       (let ((goal-position (send (elt (send goal :goal :trajectory :points) 1) :positions)) ;; goal
             (odom-pos (scale 0.001 (send (send self :state :odom :pose) :pos)))
             (odom-angle (elt (car (send (send self :state :odom :pose) :rpy-angle)) 0))
             diff-position v)
         (setq diff-position (v- goal-position (v+ odom-pos (float-vector 0 0 odom-angle))))
         (setq v (v- (rotate-vector (float-vector (elt diff-position 0) (elt diff-position 1) 0)
                                    (- odom-angle) :z)
                     (float-vector 0 0 odom-angle)))
         (setq x (elt v 0)
               y (elt v 1)
               d (rad2deg (elt diff-position 2)))
         (if (> d (* 360 0.8)) (setq d (- d 360)))
         (if (< d (* -360 0.8)) (setq d (+ d 360)))
         (ros::ros-info "                diff-pos ~A ~A, diff-angle ~A" x y d)
         )
       (incf counter)
       ) ;; while
     t)))

(pr2-init)

;;move-fridge-traj little change
(defun move-fridge-traj (hand cds-traj
                              ;; &key ((:rotation-axis ra) :z) (use-torso 0.0025)
                              &key ((:rotation-axis ra) :z) (use-torso nil)
                              (fix-waist-z 130) (return-sequence) (wait-time 0)
                              (grasp-check t)  (move-robot t) (start-sec 1000) (func-time 20) &allow-other-keys)
  (let* ((st0 (car cds-traj))
	 (st1 (cadr cds-traj))
	 (lst (cddr cds-traj))
	 (time-tick (/ (* (- func-time 16) 1000) (length lst)));;changing opening-door speed
	 avs tms)
    (when move-robot  ;; open-gripper
      (send *ri* :move-gripper hand 0.09 :wait nil))
    (when fix-waist-z
      (unless (numberp fix-waist-z) (setq fix-waist-z 130))
      (send *pr2* :torso :waist-z :joint-angle fix-waist-z))
    ;; (cond
    ;;  ((move-arm hand st0
    ;;             :use-torso (if use-torso (/ use-torso 5))
    ;;             :rotation-axis t :sec start-sec :move-robot move-robot)
    ;;   t)
    ;;  (t
    ;;   (ros::ros-warn "Could not solve ik, return from execute-open")
    ;;   (return-from move-fridge-traj nil)))

    ;; (move-arm hand st1
    ;;           :use-torso (if use-torso (/ use-torso 5))
    ;;           :rotation-axis t :sec start-sec :move-robot move-robot)

    (send *ri* :start-grasp hand :gain 0.1)
    ;; (let ((grasp-ret (if move-robot (send *ri* :start-grasp hand :gain 0.1) 10)))
    ;;   (when (and grasp-check
    ;;              (< grasp-ret 8)) ;; grasp
    ;;     (ros::ros-warn "Grasp handle failed, return from execute-open")
    ;;     (return-from move-fridge-traj nil)))

    (dolist (rr lst)
      (let ((ik-solved
             (send *pr2* :inverse-kinematics
                   (send rr :worldcoords)
                   :use-torso use-torso
                   :move-arm hand :rotation-axis ra :debug-view nil :warnp nil)))
        (unless (or ik-solved (eq ra nil))
          (warn ";; retry ~A~%" rr);
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (send rr :copy-worldcoords)
                      :use-torso use-torso
                      :move-arm hand :rotation-axis nil :debug-view nil :warnp nil)))
        (unless ik-solved (warn ";; failed ~A~%" rr))
        (when ik-solved
          (push (send *pr2* :angle-vector) avs)
          (push time-tick tms))
        ))
    (when return-sequence
      (return-from move-fridge-traj (list (reverse avs) tms)))
;;;;;;追加
    (let ((grasp-ret (if move-robot (send *ri* :get-grasp-result) 10)))
      (when (and grasp-check
                 (< grasp-ret 8)) ;; grasp
        (ros::ros-warn "Grasp handle failed, return from execute-open")
        (return-from move-fridge-traj nil)))
;;;;;;
    (when move-robot
      ;;      (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
      (send *ri* :angle-vector-sequence (reverse avs) tms))
    ;; finalize
    ;; (cond
    ;;  ((integerp wait-time)
    ;;   (send *ri* :ros-wait wait-time :spin-self t)) ;; attention-check ...
    ;;  ((numberp wait-time)
    ;;   (send *ri* :ros-wait wait-time :spin-self t) ;; attention-check ...
    ;;   ;; (unix::usleep
    ;;   ;; (round (* wait-time 1000 1000)))
    ;;   )
    ;;  (wait-time
    ;;   (send *ri* :wait-interpolation)))
    ;; (send *ri* :stop-grasp hand)
    ;; (send *ri* :wait-interpolation)
    ;; ;;
    ;; (send *ri* :ros-wait 0.0 :spin t :spin-self t) ;; attention-check ...
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;; (let ((end-pt (send *pr2* hand :end-coords :worldpos))
    ;;       idx)
    ;;   (setq idx (position (find-extream lst #'(lambda(x) (distance (send x :worldpos) end-pt)) #'<) lst))
    ;;   (ros::ros-info "idx = [~a/~a]" idx (length lst))
    ;;   (when (< idx (/ (length lst) 2))
    ;;     (return-from open-fridge-traj nil)))
    t))

(setq to nil)

(defun catch-door
  nil
  (let (target-obj move-dis via1 via2)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :stop-grasp :rarm)
    (setq target-obj (check-detection :type "fridge" :speak nil))
    (setq to target-obj)
    (setq move-dis (list (* (- (elt (send target-obj :pos) 0) 772) 0.001)
                         (* (- (elt (send target-obj :pos) 1) 98) 0.001)))
    (send target-obj :locate (float-vector 772 98 (elt (send target-obj :pos) 2)) :world)
    (setq via1 (send (send target-obj :copy-worldcoords) :translate (float-vector -100 0 0)))
    (setq via2 (send (send target-obj :copy-worldcoords) :translate (float-vector 0 0 0)))
    (send *pr2* :rarm :inverse-kinematics
          (send via1 :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :go-force (elt move-dis 0) (elt move-dis 1) 0)
    (unix:usleep 2000000)
    (send *pr2* :rarm :inverse-kinematics
          (send via2 :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (unix:usleep 400000)
;;    (send *ri* :start-grasp :rarm)
    (unix:usleep 200000)
    )
  )

(defun adjust
  nil
  (let (target-obj target-coords shouldbe target2shouldbe-trans goingto move-trans via2)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq target-obj (check-detection :type "fridge" :speak nil))
    (setq target-coords (send (send *pr2* :copy-worldcoords) :transform target-obj))
    (send target-coords :locate (float-vector (elt (send target-coords :pos) 0) (elt (send target-coords :pos) 1) 0) :world)
    (send target-coords :orient (elt (elt (rpy-angle (send target-coords :rot)) 0) 0) :z :world)
    (setq shouldbe (send (send target-coords :copy-worldcoords) :locate #f(-772 -98 0)))
    ;; (setq target2shouldbe-trans (send target-coords :transformation shouldbe))
    ;; (setq goingto (send (send target-coords :copy-worldcoords) :transform target2shouldbe-trans))
    ;; (setq move-trans (send *pr2* :transformation goingto))
    (setq move-trans (send *pr2* :transformation shouldbe))

    (setq via2 (make-coords :pos #f(772 98 1100)))
    (send *pr2* :rarm :inverse-kinematics
          (send via2 :transform (send *pr2* :worldcoords) :world)
          :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :go-force (* (elt (send move-trans :pos) 0) 0.001) (* (elt (send move-trans :pos) 1) 0.001) (elt (elt (rpy-angle (send move-trans :rot)) 0) 0))
    ))

(defun open-door
  nil
  (let (target-obj ret)
    (setq target-obj (check-detection :type "fridge" :speak nil))
    ;; (setq ret (funcall open-fridge-traj
    ;;                    :rarm target-obj 1.7
    ;;                    :rotation-axis t :radius 490
    ;;                    :wait-time 5.8 ;; t
    ;;                    ))))
    (setq ret (open-fridge-traj
                       :rarm target-obj 1.7
                       :rotation-axis t :radius 490
                       :wait-time 5.8 ;; t
                       ))))

(defun swipe (&key (use-arm :larm))
  (send *ri* :angle-vector #f(129.977 47.9019 41.7184 69.7801 -92.7089 -82.5547 -5.72958 19.9702 -59.3335 16.2856 -4.12819 -80.9055 292.353 -109.344 172.148 -19.9991 23.904))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (unix:usleep 2000000)
  (send *pr2* :larm :angle-vector (float-vector 20.0 45.0 53.0 -100.0 -43.0 -60.0 0.0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 700)
  (send *ri* :stop-grasp :rarm)
  ;; (send *ri* :wait-interpolation)
  ;; (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (unix:usleep 200000)
  (send *ri* :angle-vector-sequence
        (list
         (float-vector 130 -8.34106 16.3632 56.7586 -50.7209 -42.9818 -69.7732 -6.52931 -61.9676 12.5409 -37.3903 -42.7413 338.286 -107.454 135.477 0.027956 22.926)
         (float-vector 130 -20.2546 15.9172 53.2397 -38.2041 -43.0083 -60.0086 -0.000466 -66.8745 25.0654 -42.6273 -49.3771 319.162 -71.6589 167.523 -0.020044 23.832)
         (float-vector 130 -25.9215 15.5489 55.3988 -36.512 -44.1484 -28.2568 -7.75581 -50.5861 44.487 -45.2274 -70.2799 245.195 -26.7547 148.858 0.027956 -7.548)
         (float-vector 130 38.3106 20.1389 77.7708 -88.0224 -47.5755 -5.72958 -10.483 -12.7603 46.1883 -29.4706 -70.2965 245.815 -44.8729 150.788 0.003956 -13.794)
         )
        (list 1200 600 1000 700))
  (send *ri* :stop-grasp :rarm)
  ;;  (send *ri* :wait-interpolation)
  (unix:usleep 3400)

  (case use-arm
    (:rarm
     (send *ri* :angle-vector (float-vector 130 5.30455 69.0 105.231 -88.5188 -69.9972 -5.72958 19.9717 31.3839 25.5029 23.0531 -118.916 160.305 -84.1469 160.058 -20 24) 1500))
    (:larm
     (send *ri* :angle-vector (float-vector 130 -32.3186 26.4366 -19.6876 -118.217 -138.147 -78.3509 -166.767 -5.30455 69.0 -105.231 -88.5188 69.9972 -5.72958 -19.9717 20.0 24.0) 1500)))
  )




(defun dl
  nil
  (catch-door)
  (adjust)
  (open-door)
  (swipe :larm)
    ;; (send *ri* :go-pos-unsafe -0.5 0 0)
    ;; (send *pr2* :reset-pose)
    ;; (send *pr2* :torso :waist-z 130)
    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    ;; (send *ri* :wait-interpolation)
  )