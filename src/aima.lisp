(in-package :algorithm)

(defstruct problem states initial-state actions transition-model goal-test path-cost)

(defstruct node key value path-cost)

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
         do ;;(setf (gethash (funcall key-fn node) explored) t)
           (loop for action in (funcall actions node)
              and child = (child-node problem node action)
              when (ready-to-explore? frontierq child)
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
         if (funcall goal-test node)
         return node
         else
         do (loop for action in (funcall actions node)
               and child = (child-node problem node action)
               if (ready-to-explore? frontierq child)
               do (queue-put frontierq child)
               else
               do (let ((existing-node (peek-from-frontier frontierq child)))
                    (when (and existing-node (< (path-cost child) (path-cost existing-node)))
                      (update-frontier frontierq child))))))))
