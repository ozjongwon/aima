(asdf:defsystem #:algorithm-system
  :version "0.1.0"
  :author "Jong-won Choi(oz.jongwon.choi@gmail.com)"
  :license "MIT"
  :depends-on (#:alexandria)
  :serial t
  :components ((:module "src"
                        :components
                        ((:file "package")
                         (:file "queue" :depends-on ("package"))))))
;;
;;(asdf:compile-system :algorithm-system)
;;(asdf:load-system :algorithm-system)
;;
