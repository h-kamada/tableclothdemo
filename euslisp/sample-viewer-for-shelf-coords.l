
;; arrow
;; red - x
;; green - y
;; blue - z


;; b(60度傾いてる)


;;         c(いるべき場所)




;; a



(require "models/arrow-1000-object")
(require "models/arrow-object")

(setq *expected-dis* (send (make-coords) :translate (float-vector 1000 0 0)))

(setq a (arrow-1000))
(setq b (send (arrow) :translate (float-vector 3000 0 0)))
(send b :rotate (deg2rad 60) :z)

(setq c (send (arrow) :move-to b))
(send c :translate (v- (float-vector 0 0 0) (send *expected-dis* :pos)))

(ros::ros-info "move:~A" (send a :transformation c))
(ros::ros-info "difference-rotation:~A" (send a :difference-rotation c))


(objects (list a b c))