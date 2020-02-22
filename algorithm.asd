(asdf:defsystem #:algorithm-system
  :version "0.1.0"
  :author "Jong-won Choi(oz.jongwon.choi@gmail.com)"
  :license "MIT"
  :depends-on (#:alexandria)
  :serial t
  :components ((:module "src"
                        :components
                        ((:file "package")
                         (:file "utils" :depends-on ("package"))
                         (:file "queue" :depends-on ("utils"))
                         (:file "aima" :depends-on ("queue"))))))
;;
;;(asdf:compile-system :algorithm-system)
;;(asdf:load-system :algorithm-system)
;;
