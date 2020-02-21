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
    (make-node :state (state+action->next-state problem parent-state action)
               :parent parent
               :action action
               :path-cost (+ path-cost (state-transition-cost problem parent-state action)))))

(defstruct (frontier-queue (:constructor %make-frontier-queue))
  (queue nil :type (or heap fibonacci-heap fifo lifo))
  (frontier (make-hash-table) :type hash-table)
  (explored (make-hash-table) :type hash-table))

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

(defun uniform-cost-search (problem)
  (let ((frontierq (make-frontier-queue :fibonacci-heap)))
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

(defun solve-maze (algorithm)
  (let ((maze (make-simple-maze :algorithm algorithm :initial-state 's :goal-test 'g)))
    (ecase algorithm
      (:fifo (breadth-first-search maze))
      (:lifo (depth-first-search maze))
      ((:fibonacci-heap :heap) (uniform-cost-search maze)))))

(defmethod state+action->next-state ((problem simple-maze) state action)
  ;; action == position index
  (when-let ((found (assoc state +maze-map+)))
    (car (nth action (second found)))))

(defmethod state-transition-cost ((problem simple-maze) state action)
  (with-slots (algorithm)
      problem
    (if (member algorithm '(:fifo :lifo))
        (let ((found (assoc state +maze-map+)))
          (cdr (nth action (second found))))
        0)))

(defmethod applicable-actions ((problem simple-maze) state)
  (let ((found (assoc state +maze-map+)))
    (with-slots (algorithm)
        problem
      (let ((indexes (range (length (second found)))))
        (if (eq algorithm :lifo)
            (reverse indexes)
            indexes)))))

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

;; (depth-first-search *simple-maze-dfs*)
;; (breadth-first-search *simple-maze-bfs*)
;; (uniform-cost-search *simple-maze-ucs*)
