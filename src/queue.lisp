;;; -*- Mode: Lisp; Syntax: Common-Lisp; -*-

(in-package :algorithm)

;;;
;;; Heap
;;;
;;; From Algorithms(Robert Sedgewick and Kevin Wayne)
;;;
(defstruct (heap (:constructor %make-heap))
  (n 0 :type fixnum)
  (vector #() :type vector)
  (comp-fn #'< :type function)
  (key-fn #'identity :type function))

(defun %compare-elements-by-indexes (heap idx1 idx2)
  (with-slots (vector comp-fn key-fn)
      heap
    (funcall comp-fn
             (funcall key-fn (svref vector idx1))
             (funcall key-fn (svref vector idx2)))))

(defun %swim (heap k)
  (with-slots (vector comp-fn key-fn)
      heap
    (loop for idx = k then parent-idx
       and parent-idx = (floor (1- k) 2) then (floor (1- parent-idx) 2)
       while (and (plusp idx)
                  (%compare-elements-by-indexes heap parent-idx idx))
       do
         (rotatef (svref vector parent-idx) (svref vector idx)))))

;;(setf h1 (make-queue :heap :min-max-key :min :key-fn #'identity))

(defun %sink (heap k)
  (with-slots (n vector)
      heap
    (loop for idx = k then child-idx
       and child-idx = (floor (1+ (* k 2))) then (1+ (* child-idx 2))
       as rchild-idx = (1+ child-idx)
       while (< child-idx n)
       when (and (< rchild-idx n)
                 (%compare-elements-by-indexes heap child-idx rchild-idx))
       do (setf child-idx rchild-idx)
       when (%compare-elements-by-indexes heap idx child-idx)
       do (rotatef (svref vector idx) (svref vector child-idx)))))

(defun %resize (heap capacity)
  (with-slots (n vector)
      heap
    (assert (< n capacity))
    (let ((new-vector (make-array (list capacity))))
      (dotimes (i n)
        (setf (svref new-vector i) (svref vector i)))
      (setf (heap-vector heap) new-vector))))

;;;
;;; Public API
;;;
(defmethod queue-empty? ((heap heap))
  (with-slots (n)
      heap
    (zerop n)))

(defmethod queue-put ((heap heap) x)
  (with-slots (n vector)
      heap
    (let ((len (length vector)))
      (when (= n len)
        (%resize heap (* 2 len))))
    (with-slots (vector)
        heap
      (setf (svref vector n) x)
      (%swim heap n)
      (incf n))))

(defmethod queue-get ((heap heap))
  (assert (not (queue-empty? heap)) nil "Priority queue underflow")
  (with-slots (n vector)
      heap
    (let ((top-elem (svref vector 0))
          (vlen (length vector)))
      (decf n)
      (rotatef (svref vector 0) (svref vector n))
      (%sink heap 0)
      (setf (svref vector n) 0)

      (when (and (plusp n) (= n (floor vlen 4)))
        (%resize heap (floor vlen 2)))
      top-elem)))

;; (setf h1 (make-queue :heap :min-max-key :max :key-fn #'identity))
;; (dolist (elem '("O" "R" "T" "E" "X" "A" "M" "P" "L" "E"))
;;   (queue-put h1 elem))

;;;
;;; Fibonacci Heap
;;;
;;; From Introduction to Algorithms(Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, and Clifford Stein)
;;; 
;;;
;;; A node has one 'child' link - a main entry point for all of its siblings
;;; and the child has 'left' and 'right' links to its siblings.
;;;
;;; Call 'child' node as 'rightmost node', and 'right' linked node of the rightmost node is
;;; 'leftmost node'

;;;
;;; Low level structures & functions
;;;
(defconstant +golden-ratio+ (/ (1+ (sqrt 5)) 2))
(defun %upper-bound (n)
  (floor (log n +golden-ratio+)))

(defstruct (fibonacci-heap (:constructor %make-fibonacci-heap))
  (root nil :type (or fibonacci-node null)) ; minimum node, min, for example
  (n 0 :type fixnum)                        ; number of nodes
  (comp-fn #'< :type function)
  (key-fn #'identity :type function))

(defun %compare-nodes (heap node1 node2)
  (with-slots (comp-fn key-fn)
      heap
    (funcall comp-fn
             (funcall key-fn (fibonacci-node-elem node1))
             (funcall key-fn (fibonacci-node-elem node2)))))

(defstruct (fibonacci-node (:constructor %make-fibonacci-node))
  (elem)
  (parent nil :type (or fibonacci-node null))
  (degree 0 :type fixnum)               ; number of children
  (child nil :type (or fibonacci-node null))
  (left nil :type (or fibonacci-node null))
  (right nil :type (or fibonacci-node null)))

(defun make-fibonacci-node (elem)
  (let ((node (%make-fibonacci-node :elem elem)))
    (setf (fibonacci-node-left node) node
          (fibonacci-node-right node) node)
    node))

(defun %unlink-siblings! (node)
  (setf (fibonacci-node-left node) node
        (fibonacci-node-right node) node))

(defun %break-node-from-sibling-links! (node)
  (let ((node-l (fibonacci-node-left node))
        (node-r (fibonacci-node-right node)))
    (setf (fibonacci-node-right node-l) node-r
          (fibonacci-node-left node-r) node-l))
  (%unlink-siblings! node))

(defun %break-all-node-links! (node)
  (%break-node-from-sibling-links! node)
  (setf (fibonacci-node-child node) nil
        (fibonacci-node-degree node) 0))

(defun %insert-node-to-sibling-list! (existing-node node-r)
  (let ((node-l (fibonacci-node-right node-r))
        (existing-node-r (fibonacci-node-right existing-node)))
    (setf (fibonacci-node-right existing-node) node-l
          (fibonacci-node-left node-l) existing-node
          (fibonacci-node-left existing-node-r) node-r
          (fibonacci-node-right node-r) existing-node-r)))

(defun %insert-node-to-root-list! (heap node)
  (with-slots (root key-fn)
      heap
    (if root
        (progn
          (%insert-node-to-sibling-list! root node)
          (when (%compare-nodes heap root node)
            (setf root node)))
        (progn ;; root is empty
          (%unlink-siblings! node)
          (setf root node)))))

(defun %node->sibling-list (start-node)
  (loop for next-node = (fibonacci-node-right start-node) then (fibonacci-node-right next-node)
     and node = start-node then next-node
     collect node
     until (eq next-node start-node)))

(defun %link! (heap child parent)
  (declare (ignorable heap)) ;; child can be removed from root list without using heap
  (%break-node-from-sibling-links! child)
  (let ((rightmost-child (fibonacci-node-child parent)))
    (setf (fibonacci-node-parent child) parent)
    (if rightmost-child
        (%insert-node-to-sibling-list! rightmost-child child)
        (setf (fibonacci-node-child parent) child))
    (incf (fibonacci-node-degree parent)))
  nil)

(defun %consolidate (heap)
  (with-slots (n root)
      heap
    (let ((degree-vector (make-array (list (%upper-bound n)) :initial-element nil)))
      (loop for node in (%node->sibling-list root)
         do
           (let ((degree (fibonacci-node-degree node)))
             (loop as existing-node = (svref degree-vector degree)
                while existing-node
                do
                  (when (%compare-nodes heap node existing-node)
                    (rotatef node existing-node)) ;; assume var 'node' is 'parent'

                  (%link! heap existing-node node) ; establish parent-child link 'node'- 'existing-node'
                  (setf (svref degree-vector degree) nil)
                  (incf degree))
             (unless (%compare-nodes heap node root)
               (setf root node))
             (setf (svref degree-vector degree) node))))))

;;;
;;; Public API
;;;
(defun make-queue (queue-type &key min-max-key key-fn)
  (let ((comp-fn (if (eq :min min-max-key)
                     #'>
                     #'<)))
    (ecase queue-type
      (:fifo (%make-fifo))
      (:lifo (%make-lifo))
      (:heap (%make-heap :n 0 :vector (vector nil) :comp-fn comp-fn :key-fn key-fn))
      (:fibonacci-heap (%make-fibonacci-heap :comp-fn comp-fn :key-fn key-fn)))))

(defmethod queue-empty? ((heap fibonacci-heap))
  (with-slots (n)
      heap
    (zerop n)))

(defmethod queue-put ((heap fibonacci-heap) elem)
  (let ((node (make-fibonacci-node elem)))
    (%insert-node-to-root-list! heap node)
    (incf (fibonacci-heap-n heap))))

(defmethod queue-get ((heap fibonacci-heap))
  (let ((root (fibonacci-heap-root heap)))
    (when root
      (let ((root-r (fibonacci-node-right root))
            (child-start (fibonacci-node-child root)))
        (when child-start
          (%insert-node-to-root-list! heap child-start))
        (%break-all-node-links! root)
        (setf (fibonacci-heap-root heap) (if (eq root root-r)
                                             child-start
                                             root-r))
        (when (fibonacci-heap-root heap)
          (%consolidate heap)))
      (decf (fibonacci-heap-n heap))

      (setf (fibonacci-node-parent root) nil
            (fibonacci-node-degree root) 0))
    (fibonacci-node-elem root)))

;;;
;;; FIFO
;;;
(defstruct (fifo (:constructor %make-fifo))
  (n 0 :type fixnum)
  (list () :type list))

(defmethod queue-empty? ((queue fifo))
  (null (fifo-list queue)))

(defmethod queue-put ((queue fifo) elem)
  (with-slots (n list)
      queue
    (setf list (nconc list (list elem)))
    (incf n)))

(defmethod queue-get ((queue fifo))
  (with-slots (n list)
      queue
    (assert (plusp n) nil "FIFO underflow")
    (decf n)
    (pop list)))

;;;
;;; LIFO
;;;
(defstruct (lifo (:constructor %make-lifo))
  (n 0 :type fixnum)
  (list () :type list))

(defmethod queue-empty? ((stack lifo))
  (null (lifo-list stack)))

(defmethod queue-put ((stack lifo) elem)
  (with-slots (n list)
      stack
    (push elem list)
    (incf n)))

(defmethod queue-get ((stack lifo))
  (with-slots (n list)
      stack
    (assert (plusp n) nil "LIFO underflow")
    (decf n)
    (pop list)))


;; (setf fh1 (make-queue :fibonacci-heap :min-max-key :max :key-fn #'identity))
;; (dolist (elem (list 23 7 21 3 18 52 38 39 41 17 30 24 26 46 35))
;;   (queue-put fh1 elem))
;;
;; (list 3 7 17 18 21 23 24 26 30 35 38 39 41 46 52)
;; (loop as elem = (queue-get fh1)
;;    collect elem
;;    while (not (queue-empty? fh1)))

;;;
;;;              Binary Heap   Vs.   Fibonacci Heap
;;; queue-put        lg n                  1
;;; queue-get        lg n                lg n
;;;
;;; * queue-put (Ex only. Array resizing, copying, etc)
;;;                             Time                                            Space
;;; 1000        4,358           261                                     16,688          64,000
;;; 10000       22,898          2,641                                   262,576         640,000
;;; 100000      289,002         37,726                                  2,097,680       6,400,000
;;; 1000000     3,300,700       453,950                                 16,777,840      64,000,000
;;;
;;; * queue-get (Ex only. Array resizing, copying, etc)
;;;
;;; 1000        5,823           5,823                                   8,464           328,928
;;; 10000       33,449          28,878                                  131,472         3,924,016
;;; 100000      455,367         234,616                                 1,049,072       45,614,992
;;; 1000000     5,580,898       2,489,544                               8,389,200       523,762,144
;;;

;; (defun time-queue-put-get ()
;;   (labels ((heap-n-put-get (heap n)
;;              (format t "~&****** ~D~&> PUT~&" n)
;;              (time (dotimes (i n)
;;                      (queue-put heap i)))
;;              (format t "> GET~&")
;;              (time (dotimes (i n)
;;                      (queue-get heap)))))
;;     (heap-n-put-get (make-queue :heap :min-max-key :min :key-fn #'identity) 1000)
;;     (heap-n-put-get (make-queue :fibonacci-heap :min-max-key :min :key-fn #'identity) 1000)
;;     (heap-n-put-get (make-queue :heap :min-max-key :min :key-fn #'identity) 10000)
;;     (heap-n-put-get (make-queue :fibonacci-heap :min-max-key :min :key-fn #'identity) 10000)
;;     (heap-n-put-get (make-queue :heap :min-max-key :min :key-fn #'identity) 100000)
;;     (heap-n-put-get (make-queue :fibonacci-heap :min-max-key :min :key-fn #'identity) 100000)
;;     (heap-n-put-get (make-queue :heap :min-max-key :min :key-fn #'identity) 1000000)
;;     (heap-n-put-get (make-queue :fibonacci-heap :min-max-key :min :key-fn #'identity) 1000000)
;;     nil))
;;
;; (defun test-queues (n)
;;   (let ((q1 (make-queue :heap :min-max-key :min :key-fn #'identity))
;;         (q2 (make-queue :fibonacci-heap :min-max-key :min :key-fn #'identity))
;;         (random-list (loop repeat n
;;                           collect (random 100.0))))
;;     (dolist (x random-list)
;;       (queue-put q1 x)
;;       (queue-put q2 x))
;;     (let ((q1-copy (copy-seq (heap-vector q1))))
;;      (dotimes (i n)
;;        (let ((n1 (queue-get q1))
;;              (n2 (queue-get q2)))
;;          (assert (= n1 n2) () (list  n1 n2 random-list q1-copy)))))))

;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;
;; (ql:quickload "cl-dot")

;; (defmethod cl-dot:graph-object-node ((graph fibonacci-heap) (node fibonacci-node))
;;   (make-instance 'cl-dot:node
;;                  :attributes `(:label ,(fibonacci-node-elem node)
;;                                       :shape :circle
;;                                       :style :filled
;;                                       :fillcolor ,(cond ((eq (fibonacci-heap-root graph) node)
;;                                                          :pink)
;;                                                         (t :grey))
;;                                       :fontcolor :black)))

;; ;; Edges and their attributes
;; (defconstant +right-link+  '(:color :blue :style :bold))
;; (defconstant +left-link+  '(:color :red  :style :bold))
;; (defconstant +main-child-link+  '(:color :purple  :style :dashed))
;; (defconstant +child-link+  '(:color :violet  :style :dotted))

;; (defmethod cl-dot:graph-object-points-to ((graph fibonacci-heap) (node fibonacci-node))
;;   (let ((node-lr (list (make-instance 'cl-dot:attributed
;;                                       :object (fibonacci-node-left node)
;;                                       :attributes +left-link+)
;;                        (make-instance 'cl-dot:attributed
;;                                       :object (fibonacci-node-right node)
;;                                       :attributes +right-link+)))
;;         (child (fibonacci-node-child node)))
;;     (if child
;;         (append node-lr (list (make-instance 'cl-dot:attributed
;;                                              :object (fibonacci-node-child node)
;;                                              :attributes +main-child-link+)))
;;         node-lr)))


;; (defun emit-fheap-graph (heap &optional node)
;;   (cl-dot:dot-graph (cl-dot:generate-graph-from-roots heap
;;                                                       (%node->sibling-list (or node (fibonacci-heap-root heap)))
;;                                                       '(:rankdir "LR"))
;;                     (format nil "/tmp/fibonacchi-heap~D.png" (get-universal-time)) :format :png)
;;   (read))

