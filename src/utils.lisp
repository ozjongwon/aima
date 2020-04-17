(in-package :algorithm)

(defun range (max &key (min 0) (step 1))
  (loop for n from min below max by step
     collect n))

(defun %-> (first &rest exps)
  (reduce #'(lambda (result exp)
              (push result (cdr exp))
              exp)
          exps
          :initial-value first))

(defmacro -> (&rest exps)
  (let ((result (apply #'%-> exps)))
    `(,@result)))

(defun %->> (&rest exps)
  (reduce #'(lambda (result exp)
              (setf (cdr (last exp)) (list result))
              exp)
          exps))

(defmacro ->> (&rest exps)
  (let ((result (apply #'%->> exps)))
    `(,@result)))

;; (->> (+ 1 12) (+ 2) (+ 3))
;; (+ 3 (+ 2 (+ 1 12)))
