(in-package :algorithm)

(defun range (max &key (min 0) (step 1))
  (loop for n from min below max by step
     collect n))
