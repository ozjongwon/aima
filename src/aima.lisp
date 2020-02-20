(in-package :algorithm)

(defstruct problem states initial-state actions transition-model goal-test path-cost)

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

(defstruct (node (:constructor %make-node))
  state
  (parent nil :type (or nil node))
  (action nil :type keyword)
  (path-cost most-positive-fixnum :type fixnum))

(defun child-node (problem parent action)
  (with-slots ((parent-state state) path-cost)
      parent
    (%make-node :state (result problem parent-state action)
                :parent parent
                :action action
                :path-cost (+ path-cost (step-cost problem parent-state action)))))

(defstruct (frontier-queue (:constructor %make-frontier-queue))
  (queue nil :type (or heap fibonacci-heap fifo lifo))
  (frontier (make-hash-table) :type hash-table)
  (explored (make-hash-table) :type hash-table)
  (key-fn nil :type (or nil function)))

(defmethod queue-empty? ((fq frontier-queue))
  (with-slots (queue)
      fq
    (queue-empty? queue)))

(defmethod queue-put ((fq frontier-queue) elem)
  (with-slots (queue frontier key-fn)
      fq
    (let ((elem-key (funcall key-fn elem)))
      (queue-put queue elem-key)
      (setf (gethash elem-key frontier) elem))))

(defmethod queue-get ((fq frontier-queue))
  (with-slots (queue frontier explored key-fn)
      fq
    (let* ((elem-key (queue-get queue))
           (elem (gethash elem-key explored)))
      (remhash elem-key frontier)
      (setf (gethash elem-key explored) t)
      elem)))

(defmethod ready-to-explore? ((fq frontier-queue) elem)
  (with-slots (frontier explored key-fn)
      fq
    (let ((key (funcall key-fn elem)))
      (not (or (gethash key frontier) (gethash key explored))))))

(defmethod peek-from-frontier ((fq frontier-queue) elem)
  (with-slots (frontier key-fn)
      fq
    (gethash (funcall key-fn elem) frontier)))

(defmethod update-frontier ((fq frontier-queue) new)
  (with-slots (frontier key-fn)
      fq
    (let ((key (funcall key-fn new)))
     (setf (gethash key frontier) new))))

(defun make-frontier-queue (queue-type &key key-fn)
  (%make-frontier-queue :queue (make-queue queue-type :key-fn key-fn)
                        :key-fn key-fn))

(defun general-search (problem queue-type key-fn)
  (assert (member queue-type '(:fifo :lifo)))
  (with-slots (initial-state goal-test actions)
      problem
    (when (funcall goal-test initial-state)
      (return-from general-search initial-state))
    (let ((frontierq (make-frontier-queue queue-type :key-fn key-fn)))
      (queue-put frontierq initial-state)
      (loop until (queue-empty? frontierq)
         as node = (queue-get frontierq)
         do (loop for action in (funcall actions node)
               and child = (child-node problem node action)
               when (ready-to-explore? frontierq child)
               ;; Goal test immediately before put into frontier
               do (if (funcall goal-test child)
                      (return-from general-search child)
                      (queue-put frontierq child)))))))

(defun breadth-first-search (problem key-fn)
  (general-search problem :fifo key-fn))

(defun depth-first-search (problem key-fn)
  (general-search problem :lifo key-fn))

(defun uniform-cost-search (problem key-fn)
  (with-slots (initial-state goal-test actions)
      problem
    (let ((frontierq (make-frontier-queue :fibonacci-heap :key-fn key-fn)))
      (queue-put frontierq initial-state)
      (loop until (queue-empty? frontierq)
         as node = (queue-get frontierq)
         ;; Goal test when frontier expands
         if (funcall goal-test node)
         return node
         else
         do (loop for action in (funcall actions node)
               and child = (child-node problem node action)
               if (ready-to-explore? frontierq child)
               do (queue-put frontierq child)
               else
               do (let ((existing-node (peek-from-frontier frontierq child)))
                    (when (and existing-node (< (node-path-cost child) (node-path-cost existing-node)))
                      (update-frontier frontierq child))))))))
