(in-package :algorithm)

(defstruct problem
  states
  initial-state
  actions
  transition-model
  goal-test
  (path-cost 0 :type fixnum))

(defgeneric state+action->next-state (problem state action)
  (:documentation "RESULT function in AIMA."))
#+XXX
(defmethod state+action->next-state ((problem problem) state action)
  (with-slots (transition-model)
      problem
    ;; generate the next state
    state action
    ))

(defgeneric state-transition-cost (problem state action)
  (:documentation "STEP-COST function in AIMA."))
#+XXX
(defmethod state-transition-cost ((problem problem) state action)
  (with-slots (path-cost)
      problem
    ;; compute a step cost of sate + action
    state action
    ))

(defgeneric heuristic-function (problem node action child-state)
  (:documentation "Heuristic function for A*. 0 for non A* algorithms")
  (:method (problem node action child-state)
    (declare (ignore problem node action child-state))
    0))

(defgeneric applicable-actions (problem state)
  (:documentation "ACTIONS function in AIMA."))
#+XXX
(defmethod applicable-actions ((problem problem) state)
  (with-slots (actions)
      problem)
  ;; compute available actions
  state
  )

(defgeneric state-satisfies-goal? (problem state)
  (:documentation "GOAL-TEST function in AIMA."))
#+XXX
(defmethod state-satisfies-goal? ((problem problem) state)
  (with-slots (goal-test)
      problem
    ;;compute true or false?
    state))

(defgeneric solution (problem end-node)
  (:documentation "SOLUTION function in AIMA."))

(defstruct node
  state
  (parent nil :type (or null node))
  action
  (path-cost 0 :type fixnum))

(defun child-node (problem parent action &optional search-type)
  (with-slots ((parent-state state) path-cost)
      parent
    (let ((child-state (state+action->next-state problem parent-state action)))
      (make-node :state child-state
                 :parent parent
                 :action action
                 ;; For uniform-cost-search, this is g(n)
                 ;; A* f(n) = g(n) + h(n)
                 :path-cost (+ path-cost
                               ;; g(n)
                               (state-transition-cost problem parent-state action)
                               ;; h(n), for non A* searches, 0
                               (if (member search-type '(:a* :ida*))
                                   (heuristic-function problem parent action child-state)
                                   0))))))

(defstruct (frontier-queue (:constructor %make-frontier-queue))
  (queue nil :type (or heap fibonacci-heap fifo lifo))
  (frontier (make-hash-table :test #'equalp) :type hash-table)
  (explored (make-hash-table :test #'equalp) :type hash-table))

(defmethod queue-empty? ((fq frontier-queue))
  (with-slots (queue)
      fq
    (queue-empty? queue)))

(defmethod queue-put ((fq frontier-queue) node)
  (with-slots (queue frontier)
      fq
    (queue-put queue node)
    (setf (gethash (node-state node) frontier) node)))

(defmethod queue-get ((fq frontier-queue))
  (with-slots (queue frontier explored)
      fq
    (let* ((node (queue-get queue))
           (state (node-state node)))
      (remhash state frontier)
      (setf (gethash state explored) t)
      node)))

(defmethod ready-to-explore? ((fq frontier-queue) elem)
  (with-slots (frontier explored)
      fq
    (let ((state (node-state elem)))
      (not (or (gethash state frontier) (gethash state explored))))))

(defmethod peek-frontier-node-by-state ((fq frontier-queue) state)
  (with-slots (frontier)
      fq
    (gethash state frontier)))

(defmethod update-existing-frontier-node (existing-node new-node)
  (setf (node-parent existing-node)     (node-parent new-node)
        (node-action existing-node)     (node-action new-node)
        (node-path-cost existing-node)  (node-path-cost new-node)))

(defun make-frontier-queue (queue-type)
  (%make-frontier-queue :queue (make-queue queue-type :min-max-key :min :key-fn #'node-path-cost)))

(defun general-search (problem queue-type)
  (assert (member queue-type '(:fifo :lifo)))
  (with-slots (initial-state)
      problem
    (let ((node (make-node :state initial-state)))
      (when (state-satisfies-goal? problem initial-state)
        (return-from general-search (solution problem node)))
      (let ((frontierq (make-frontier-queue queue-type)))
        (queue-put frontierq node)
        (loop until (queue-empty? frontierq)
           as node = (queue-get frontierq)
           do (loop for action in (applicable-actions problem (node-state node))
                 as child = (child-node problem node action)
                 when (ready-to-explore? frontierq child)
                 ;; Goal test immediately before put into frontier
                 do (if (state-satisfies-goal? problem (node-state child))
                        (return-from general-search (solution problem child))
                        (queue-put frontierq child))))))))

(defun breadth-first-search (problem)
  (general-search problem :fifo))

(defun depth-first-search (problem)
  (general-search problem :lifo))

(defun cost-based-search (problem search-type queue-type)
  ;; queue-type: :fibonacci-heap, :heap
  (let ((frontierq (make-frontier-queue queue-type)))
    (queue-put frontierq (make-node :state (problem-initial-state problem)))
    (loop until (queue-empty? frontierq)
       as node = (queue-get frontierq)
       ;; Goal test when frontier expands
       if (state-satisfies-goal? problem (node-state node))
       return (solution problem node)
       else
       do (loop for action in (applicable-actions problem (node-state node))
             as child = (child-node problem node action search-type)
             if (ready-to-explore? frontierq child)
             do (queue-put frontierq child)
             else
             do (when-let ((existing-node (peek-frontier-node-by-state frontierq (node-state child))))
                  (when (< (node-path-cost child) (node-path-cost existing-node))
                    (update-existing-frontier-node existing-node child)))))))

;;;;
;;;;
;;;;
(defun depth-limited-search (problem limit)
  (recursive-dls (make-node :state (problem-initial-state problem)) problem limit))

(defun recursive-dls (node problem limit)
  (cond ((state-satisfies-goal? problem (node-state node))
         (solution problem node))
        ((zerop limit) :cutoff)
        (t (loop with cutoff-occurred? = nil
              for action in (applicable-actions problem (node-state node))
              as child = (child-node problem node action)
              as result = (recursive-dls child problem (1- limit))
              if (eq result :cutoff)
              do (setf cutoff-occurred? t)
              else when result return result
              finally (when cutoff-occurred?
                        (return-from recursive-dls :cutoff))))))

(defun iterative-deepening-search (problem)
  (loop for depth = 0 then (incf depth)
     as result = (depth-limited-search problem depth)
     unless (eq result :cutoff)
     return result))

;;;;;;;;;;;;;
(defconstant +maze-map+
  '((s ((a . 6) (b . 2) (c . 5)))
    (a ((d . 9)))
    (b ((e . 3)))
    (c ((h . 2)))
    (d)
    (e ((a . 2)))
    (f ((d . 4)))
    (g ((d . 1) (e . 5)))
    (h ((f . 2) (g . 7)))))

(defstruct (simple-maze (:include problem))
  (search-type nil :type keyword))

(defconstant +romania-map+
  '(
    (Arad      ((Zerind . 75) (Sibiu . 140) (Timisoara . 118)))
    (Bucharest ((Fagaras . 211) (Pitesti . 101) (Giurgiu . 90)
                (Urziceni . 85)))
    (Craiova   ((Dobreta . 120) (Rimnicu . 146) (Pitesti . 138)))
    (Dobreta   ((Mehadia . 75) (Craiova . 120)))
    (Eforie    ((Hirsova . 86)))
    (Fagaras   ((Sibiu . 99) (Bucharest . 211)))
    (Giurgiu   ((Bucharest . 90)))
    (Hirsova   ((Urziceni . 98) (Eforie . 86)))
    (Iasi      ((Neamt . 87) (Vaslui . 92)))
    (Lugoj     ((Timisoara . 111) (Mehadia . 70)))
    (Mehadia   ((Lugoj . 70) (Dobreta . 75)))
    (Neamt     ((Iasi . 87)))
    (Oradea    ((Zerind . 71) (Sibiu . 151)))
    (Pitesti   ((Rimnicu . 97) (Craiova . 138) (Bucharest . 101)))
    (Rimnicu   ((Sibiu . 80) (Pitesti . 97) (Craiova . 146)))
    (Sibiu     ((Arad . 140) (Oradea . 151) (Fagaras . 99)
                (Rimnicu . 80)))
    (Timisoara ((Arad . 118) (Lugoj . 111)))
    (Urziceni  ((Bucharest . 85) (Hirsova . 98) (Vaslui . 142)))
    (Vaslui    ((Iasi . 92) (Urziceni . 142)))
    (Zerind    ((Arad . 75) (Oradea . 71)))
    )
  "A representation of the map in Figure 4.2 [p 95].
  But note that the straight-line distances to Bucharest are NOT the same.")

(defun solve-simple-maze (search-type map start end &optional queue-type)
  (let ((maze (make-simple-maze :search-type search-type :initial-state start :goal-test end :transition-model map)))
    (ecase search-type
      (:breadth-first (breadth-first-search maze))
      (:depth-first (depth-first-search maze))
      ((:uniform-cost :a*) (cost-based-search maze search-type queue-type))
      (:rbfs (recursive-best-first-search maze queue-type))
      (:ids (iterative-deepening-search maze))
      (:ida* (ida-star-search maze)))))

(defun get-state-neighbours (transition-model state)
  (first (last (assoc state transition-model))))

(defun pick-state-neighbour-info (transition-model state idx)
  (when-let ((found (get-state-neighbours transition-model state)))
    (nth idx found)))

(defun path->cost (map path)
  (loop for start in (butlast path)
     and end in (rest path)
     as neighbours = (get-state-neighbours map start)
     sum (cdr (assoc end neighbours))))

(defmethod state+action->next-state ((problem simple-maze) state action)
  ;; action == position index
  (with-slots (transition-model)
      problem
    (car (pick-state-neighbour-info transition-model state action))))

(defmethod state-transition-cost ((problem simple-maze) state action)
  (with-slots (search-type transition-model)
      problem
    (if (member search-type '(:breadth-first :depth-first))
        0
        (cdr (pick-state-neighbour-info transition-model state action)))))

(defmethod applicable-actions ((problem simple-maze) state)
  (with-slots (transition-model)
      problem
    (range (length (get-state-neighbours transition-model state)))))

(defmethod state-satisfies-goal? ((problem simple-maze) state)
  (with-slots (goal-test)
      problem
    (eq goal-test state)))

(defmethod solution ((problem simple-maze) end-node)
  (loop with states = ()
     for node = end-node then (node-parent node)
     while node
     do (push (node-state node) states)
     finally (return states)))

(defstruct (n-puzzle (:include problem))
  (sqrt-n 0 :type fixnum))

(defun n-puzzle-state (l)
  (let ((len (length l)))
   (assert (integerp (sqrt len)))
   (make-array (list len) :initial-contents l)))
;; (n-puzzle-state '(1 2 5 3 4 0 6 7 8))

(defun solve-n-puzzle (search-type start &optional queue-type)
  (let* ((len (length start))
         (puzzle (make-n-puzzle :initial-state start :sqrt-n (sqrt len)
                                :goal-test (n-puzzle-state (range len)))))
    (ecase search-type
      (:breadth-first (breadth-first-search puzzle))
      (:depth-first (depth-first-search puzzle))
      ((:uniform-cost :a*) (cost-based-search puzzle search-type queue-type))
      (:rbfs (recursive-best-first-search puzzle queue-type))
      (:ids (iterative-deepening-search puzzle))
      (:ida* (ida-star-search puzzle)))))

;;
;; (time (solve-n-puzzle :breadth-first (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :fibonacci-heap))
;; (time (solve-n-puzzle :a* (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :fibonacci-heap))
;; (time (solve-n-puzzle :uniform-cost (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :fibonacci-heap))
;; (time (solve-n-puzzle :ids (n-puzzle-state '(7 2 4 5 0 6 8 3 1))))
;; (time (solve-n-puzzle :ida* (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :fibonacci-heap))
;;

(defmethod state+action->next-state ((problem n-puzzle) state action)
  ;; action => operation on the blank, 0
  (with-slots (sqrt-n)
      problem
    (let ((next-state (copy-sequence 'vector state))
          (idx (position 0 state)))
      (rotatef (svref next-state idx) (svref next-state (ecase action
                                                          (:down (+ idx sqrt-n))
                                                          (:up   (- idx sqrt-n))
                                                          (:left (1- idx))
                                                          (:right (1+ idx)))))
      next-state)))

(defmethod state-transition-cost ((problem n-puzzle) state action)
  (declare (ignore state action))
  0)

(defmethod applicable-actions ((problem n-puzzle) state)
  (with-slots (sqrt-n)
      problem
    (let* ((actions ())
           (idx (position 0 state))
           (quot (mod idx sqrt-n)))
      (when (<= sqrt-n idx)
        (push :up actions))
      (when (< idx (- (length state) sqrt-n))
        (push :down actions))
      (when (< 0 quot)
        (push :left actions))
      (when (< quot (1- sqrt-n))
        (push :right actions))
      actions)))

(defmethod state-satisfies-goal? ((problem n-puzzle) state)
  (with-slots (goal-test)
      problem
    (equalp goal-test state)))

(defmethod solution ((problem n-puzzle) end-node)
  (loop with actions = ()
     for node = end-node then (node-parent node)
     while node
     do (push (node-action node) actions)
     finally (return actions)))

(defun idx->xy (n sqrt-n)
  (let ((y (mod n sqrt-n)))
    (list (/ (- n y) sqrt-n) y)))

(defmethod heuristic-function ((problem n-puzzle) parent action child-state)
  (declare (ignore parent action))
  (with-slots (sqrt-n)
      problem
    (loop for n across child-state
       and i from 0
       ;; NOTE: don't overestimate to be admissible!
       unless (or (zerop n) (= n 1))
       ;; * strategy 1 count misplaced tiles
       ;; count 1
       sum (apply #'+ (mapcar #'(lambda (a b) ;; * strategy 2 Manhattan distance
                                  (abs (- a b)))
                              (idx->xy i sqrt-n) (idx->xy n sqrt-n))))))


;; (solve-n-puzzle :uniform-cost (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :heap)
;; (solve-n-puzzle :a* (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :heap)
;; (solve-n-puzzle :breadth-first (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :heap)
;; (solve-n-puzzle :ids (n-puzzle-state '(7 2 4 5 0 6 8 3 1)) :heap)

(defparameter +romania-map-with-coord+
  '(
    (Arad       ( 91 492) ((Zerind . 75) (Sibiu . 140) (Timisoara . 118)))
    (Bucharest	(400 327) ((Fagaras . 211) (Pitesti . 101) (Giurgiu . 90)
                           (Urziceni . 85)))
    (Craiova	(253 288) ((Dobreta . 120) (Rimnicu . 146) (Pitesti . 138)))
    (Dobreta	(165 299) ((Mehadia . 75) (Craiova . 120)))
    (Eforie	(562 293) ((Hirsova . 86)))
    (Fagaras	(305 449) ((Sibiu . 99) (Bucharest . 211)))
    (Giurgiu	(375 270) ((Bucharest . 90)))
    (Hirsova	(534 350) ((Urziceni . 98) (Eforie . 86)))
    (Iasi	(473 506) ((Neamt . 87) (Vaslui . 92)))
    (Lugoj	(165 379) ((Timisoara . 111) (Mehadia . 70)))
    (Mehadia	(168 339) ((Lugoj . 70) (Dobreta . 75)))
    (Neamt	(406 537) ((Iasi . 87)))
    (Oradea	(131 571) ((Zerind . 71) (Sibiu . 151)))
    (Pitesti	(320 368) ((Rimnicu . 97) (Craiova . 138) (Bucharest . 101)))
    (Rimnicu	(233 410) ((Sibiu . 80) (Pitesti . 97) (Craiova . 146)))
    (Sibiu	(207 457) ((Arad . 140) (Oradea . 151) (Fagaras . 99)
                           (Rimnicu . 80)))
    (Timisoara	( 94 410) ((Arad . 118) (Lugoj . 111)))
    (Urziceni	(456 350) ((Bucharest . 85) (Hirsova . 98) (Vaslui . 142)))
    (Vaslui	(509 444) ((Iasi . 92) (Urziceni . 142)))
    (Zerind	(108 531) ((Arad . 75) (Oradea . 71)))
    )
  "A representation of the map in Figure 4.2 [p 95].
  But note that the straight-line distances to Bucharest are NOT the same.")

(defstruct (romania-maze (:include simple-maze)))

(defun city-coord (map name)
  (second (assoc name map)))

(defmethod heuristic-function ((problem romania-maze) parent action child-state)
  (declare (ignore action))
  (with-slots (goal-test transition-model)
      problem
    (let* ((goal-coord (city-coord transition-model goal-test))
           (child-coord (city-coord transition-model child-state))
           (child-cost (sqrt (apply #'+ (mapcar #'(lambda (a b) (expt (- a b) 2)) goal-coord child-coord)))))
      (if parent
          (let ((parent-coord (city-coord transition-model (node-state parent))))
            (floor (- (sqrt (apply #'+ (mapcar #'(lambda (a b) (expt (- a b) 2)) goal-coord parent-coord)))
                      child-cost)))
          (floor child-cost)))))

;;
;; (time (solve-simple-maze :depth-first +romania-map-with-coord+ 'Arad 'Bucharest))
;; (time (solve-simple-maze :breadth-first +romania-map-with-coord+ 'Arad 'Bucharest))
;; (time (solve-simple-maze :uniform-cost +romania-map-with-coord+ 'Arad 'Bucharest :heap))
;; (time (solve-simple-maze :uniform-cost +romania-map-with-coord+ 'Arad 'Bucharest :fibonacci-heap))
;; (time (solve-simple-maze :a* +romania-map-with-coord+ 'Arad 'Bucharest :heap))
;; (time (solve-simple-maze :a* +romania-map-with-coord+ 'Arad 'Bucharest :fibonacci-heap))
;; (time (solve-simple-maze :ida* +romania-map-with-coord+ 'Arad 'Bucharest))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *counter* 0)

(defun recursive-best-first-search (problem queue-type)
  (setf *counter* 0)
  (rbfs problem (make-node :state (problem-initial-state problem)) most-positive-fixnum queue-type))

(defun rbfs (problem node f-limit queue-type)
  (incf *counter*)
  (when (state-satisfies-goal? problem (node-state node))
    (return-from rbfs (solution problem node)))
  (let ((node-f (node-path-cost node))
        (successors (make-queue queue-type :min-max-key :min :key-fn #'node-path-cost)))
    ;; Build successors
    (loop for action in (applicable-actions problem (node-state node))
       as successor = (child-node problem node action)
       as cost-f = (node-path-cost successor)
       when (< cost-f node-f)
       do (setf (node-path-cost successor) node-f)
       do (queue-put successors successor))
    (when (queue-empty? successors)
      (return-from rbfs `(:failure ,most-positive-fixnum)))
    ;; main
    (loop until (queue-empty? successors)
       as best = (queue-get successors)
       as best-f = (node-path-cost best)
       when (> best-f f-limit)
       return `(:failure ,best-f)
       do (when-let ((alternative (and (not (queue-empty? successors))
                                       (node-path-cost (queue-get successors)))))
            (let* ((result (rbfs problem best (min f-limit alternative) queue-type)))
              (unless (eq (first result) :failure)
                (return-from rbfs result))))
         finally (return best)
         #+XX(let* ((alternative (node-path-cost (queue-get successors)))
                    (result (rbfs problem best (min f-limit alternative) queue-type)))
               (unless (eq (first result) :failure)
                 (return-from rbfs result))))))

(defun solve-maze-heuristically (search-type map start end &optional queue-type)
  (let ((maze (make-romania-maze :search-type search-type :initial-state start :goal-test end :transition-model map)))
    (ecase search-type
      (:breadth-first (breadth-first-search maze))
      (:depth-first (depth-first-search maze))
      ((:uniform-cost :a*) (cost-based-search maze search-type queue-type))
      (:rbfs (recursive-best-first-search maze queue-type))
      (:ids (iterative-deepening-search maze))
      (:ida* (ida-star-search maze)))))

;;(time (solve-maze-heuristically :uniform-cost +romania-map-with-coord+ 'Arad 'Bucharest :heap))
;;(time (solve-maze-heuristically :a* +romania-map-with-coord+ 'Arad 'Bucharest :heap))
;;(time (solve-maze-heuristically :ids +romania-map-with-coord+ 'Arad 'Bucharest :heap))


;;;
(defun ida-star-search (problem)
  (let* ((initial-state (problem-initial-state problem)))
    (loop with f-limit = (heuristic-function problem nil nil initial-state)
       and hash-table = (make-hash-table :test #'equalp)
       with root = (let ((node (make-node :state initial-state :path-cost f-limit)))
                     (setf (gethash initial-state hash-table) t)
                     node)
       as (next-node new-f-limit) = (multiple-value-list (dfs-contour problem
                                                                      root
                                                                      f-limit
                                                                      hash-table))
       if next-node
       return (solution problem next-node)
       else if (= new-f-limit most-positive-fixnum)
       return nil
       else do (setf f-limit new-f-limit))))

(defun dfs-contour (problem node f-limit hash-table)
  (let ((f (node-path-cost node)))
    (when (> f f-limit)
      (return-from dfs-contour (values nil f))))
  (when (state-satisfies-goal? problem (node-state node))
    (return-from dfs-contour (values node f-limit)))
  (loop with next-f-limit = most-positive-fixnum
     for action in (applicable-actions problem (node-state node))
     as child = (child-node problem node action :ida*)
     as child-state = (node-state child)
     unless (gethash child-state hash-table)
     do (unless (gethash child-state hash-table)
          (setf (gethash child-state hash-table) t)
          (multiple-value-bind (next-node child-f)
              (dfs-contour problem child f-limit hash-table)
            (cond (next-node (return-from dfs-contour (values next-node child-f)))
                  ((< child-f next-f-limit) (setf next-f-limit child-f))))
          (remhash child-state hash-table))
     finally (return (values nil next-f-limit))))
