(in-package :algorithm)

(defstruct problem
  states
  initial-state
  actions
  transition-model
  goal-test
  (path-cost 0 :type fixnum))

(defmethod result ((problem problem) state action)
  (with-slots (transition-model)
      problem
    ;; generate the next state
    state action
    ))

(defmethod step-cost ((problem problem) state action)
  (with-slots (path-cost)
      problem
    ;; compute a step cost of sate + action
    state action
    ))

(defmethod actions ((problem problem) state)
  (with-slots (actions)
      problem)
  ;; compute available actions
  state
  )

(defmethod goal-test ((problem problem) state)
    (with-slots (goal-test)
        problem
      ;;compute true or false?
      state))

(defstruct node
  state
  (parent nil :type (or null node))
  (action nil :type (or null keyword))
  (path-cost 0 :type fixnum))

(defun child-node (problem parent action)
  (with-slots ((parent-state state) path-cost)
      parent
    (make-node :state (result problem parent-state action)
               :parent parent
               :action action
               :path-cost (+ path-cost (step-cost problem parent-state action)))))

(defun solution (end-node)
  (loop with actions = ()
     for node = end-node then (node-parent node)
     while node
     do (push (node-action node) actions)
     finally (return actions)))

(defstruct (frontier-queue (:constructor %make-frontier-queue))
  (queue nil :type (or heap fibonacci-heap fifo lifo))
  (frontier (make-hash-table) :type hash-table)
  (explored (make-hash-table) :type hash-table)
  (key-fn nil :type (or nil function)))

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

(defun make-frontier-queue (queue-type &key key-fn)
  (%make-frontier-queue :queue (make-queue queue-type :key-fn key-fn)
                        :key-fn key-fn))

(defun general-search (problem queue-type key-fn)
  (assert (member queue-type '(:fifo :lifo)))
  (with-slots (initial-state)
      problem
    (let ((node (make-node :state initial-state)))
      (when (goal-test problem initial-state)
        (return-from general-search (solution node)))
      (let ((frontierq (make-frontier-queue queue-type :key-fn key-fn)))
        (queue-put frontierq node)
        (loop until (queue-empty? frontierq)
           as node = (queue-get frontierq)
           do (loop for action in (actions problem (node-state node))
                 and child = (child-node problem node action)
                 when (ready-to-explore? frontierq child)
                 ;; Goal test immediately before put into frontier
                 do (if (goal-test problem (node-state child))
                        (return-from general-search child)
                        (queue-put frontierq child))))))))

(defun breadth-first-search (problem key-fn)
  (general-search problem :fifo key-fn))

(defun depth-first-search (problem key-fn)
  (general-search problem :lifo key-fn))

(defun uniform-cost-search (problem key-fn)
  (let ((frontierq (make-frontier-queue :fibonacci-heap :key-fn key-fn)))
    (queue-put frontierq (make-node :state (problem-initial-state problem)))
    (loop until (queue-empty? frontierq)
       as node = (queue-get frontierq)
       ;; Goal test when frontier expands
       if (goal-test problem (node-state node))
       return (solution node)
       else
       do (loop for action in (node-state node)
             and child = (child-node problem node action)
             if (ready-to-explore? frontierq child)
             do (queue-put frontierq child)
             else
             do (let ((existing-node (peek-frontier-node-by-state frontierq (node-state child))))
                  (when (and existing-node (< (node-path-cost child) (node-path-cost existing-node)))
                    (update-existing-frontier-node existing-node child)))))))
