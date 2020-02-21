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

(defun child-node (problem parent action)
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
                               (heuristic-function problem parent action child-state))))))

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

(defun cost-based-search (problem heap-type)
  ;; heap-type: :fibonacci-heap, :heap
  (let ((frontierq (make-frontier-queue heap-type)))
    (queue-put frontierq (make-node :state (problem-initial-state problem)))
    (loop until (queue-empty? frontierq)
       as node = (queue-get frontierq)
       ;; Goal test when frontier expands
       if (state-satisfies-goal? problem (node-state node))
       return (solution problem node)
       else
       do (loop for action in (applicable-actions problem (node-state node))
             as child = (child-node problem node action)
             if (ready-to-explore? frontierq child)
             do (queue-put frontierq child)
             else
             do (when-let ((existing-node (peek-frontier-node-by-state frontierq (node-state child))))
                  (when (< (node-path-cost child) (node-path-cost existing-node))
                    (update-existing-frontier-node existing-node child)))))))

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
  (algorithm nil :type keyword))

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

(defun solve-simple-maze (algorithm map start end)
  (let ((maze (make-simple-maze :algorithm algorithm :initial-state start :goal-test end :transition-model map)))
    (ecase algorithm
      (:fifo (breadth-first-search maze))
      (:lifo (depth-first-search maze))
      ((:fibonacci-heap :heap) (cost-based-search maze algorithm)))))


(defun get-state-neighbours (transition-model state)
  (first (last (assoc state transition-model))))

(defun pick-state-neighbour-info (transition-model state idx)
  (when-let ((found (get-state-neighbours transition-model state)))
    (nth idx found)))

(defmethod state+action->next-state ((problem simple-maze) state action)
  ;; action == position index
  (with-slots (transition-model)
      problem
    (car (pick-state-neighbour-info transition-model state action))))

(defmethod state-transition-cost ((problem simple-maze) state action)
  (with-slots (algorithm transition-model)
      problem
    (if (member algorithm '(:fifo :lifo))
        0
        (cdr (pick-state-neighbour-info transition-model state action)))))

(defmethod applicable-actions ((problem simple-maze) state)
  (with-slots (algorithm transition-model)
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

;; (solve-simple-maze :fifo +maze-map+ 's 'g)
;; (solve-simple-maze :lifo +maze-map+ 's 'g)
;; (solve-simple-maze :heap +maze-map+ 's 'g)
;; (solve-simple-maze :fibonacci-heap +maze-map+ 's 'g)
;;
;; (solve-simple-maze :fifo +romania-map+ 'Arad 'Bucharest)
;; (solve-simple-maze :lifo +romania-map+ 'Arad 'Bucharest)
;; (solve-simple-maze :heap +romania-map+ 'Arad 'Bucharest)
;; (solve-simple-maze :fibonacci-heap +romania-map+ 'Arad 'Bucharest)

(defstruct (n-puzzle (:include problem))
  (sqrt-n 0 :type fixnum))

(defun n-puzzle-state (l)
  (let ((len (length l)))
   (assert (integerp (sqrt len)))
   (make-array (list len) :initial-contents l)))
;; (n-puzzle-state '(1 2 5 3 4 0 6 7 8))

(defun solve-n-puzzle (algorithm start)
  (let* ((len (length start))
         (puzzle (make-n-puzzle :initial-state start :sqrt-n (sqrt len)
                                :goal-test (n-puzzle-state (range len)))))
    (ecase algorithm
      (:fifo (breadth-first-search puzzle))
      (:lifo (depth-first-search puzzle))
      ((:heap :fibonacci-heap) (cost-based-search puzzle algorithm)))))


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


(defmethod heuristic-function ((problem n-puzzle) parent action child-state)
  (declare (ignore parent action))
  (loop for n across child-state
     and i from 0
     count (not (= n i))))

;;(solve-n-puzzle :fifo (n-puzzle-state '(1 2 5 3 4 0 6 7 8)))
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

(defun solve-maze-heuristically (algorithm map start end)
  (let ((maze (make-romania-maze :algorithm algorithm :initial-state start :goal-test end :transition-model map)))
    (ecase algorithm
      (:fifo (breadth-first-search maze))
      (:lifo (depth-first-search maze))
      ((:fibonacci-heap :heap) (cost-based-search maze algorithm)))))

(defmethod heuristic-function ((problem romania-maze) parent action child-state)
  (declare (ignore parent action))
  (with-slots (goal-test transition-model)
      problem
    (let ((goal-coord (city-coord transition-model goal-test))
          (child-coord (city-coord transition-model child-state)))
      (round (sqrt (apply #'+ (mapcar #'(lambda (a b) (expt (- a b) 2)) goal-coord child-coord)))))))

;;
;; (time (solve-simple-maze :lifo +romania-map-with-coord+ 'Arad 'Bucharest))
;; (time (solve-simple-maze :fifo +romania-map-with-coord+ 'Arad 'Bucharest))
;; (time (solve-simple-maze :fibonacci-heap +romania-map-with-coord+ 'Arad 'Bucharest))
;; (time (solve-simple-maze :heap +romania-map-with-coord+ 'Arad 'Bucharest))
;;
