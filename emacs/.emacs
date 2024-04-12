(require 'package)
(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(custom-enabled-themes '(afternoon))
 '(custom-safe-themes
   '("1711947b59ea934e396f616b81f8be8ab98e7d57ecab649a97632339db3a3d19" default))
 '(display-time-mode t)
 '(global-display-line-numbers-mode t)
 '(inhibit-startup-screen t)
 '(package-selected-packages
   '(go-complete go-autocomplete auto-complete-c-headers cmake-mode magit afternoon-theme projectile haskell-mode rust-mode))
 '(tool-bar-mode nil)
 '(warning-suppress-log-types '((comp) (comp) (comp)))
 '(warning-suppress-types '((comp) (comp))))
(package-initialize)

(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 )

(setq-default c-basic-offset 4)

;; change <username> with your user name
(add-to-list 'load-path "/home/<username>/.emacs.d/neotree")
(require 'neotree)
(global-set-key [f8] 'neotree-toggle)
(global-set-key (kbd "C-x t") 'ff-find-other-file)

(setq org-todo-keywords '((sequence "TODO(t)" "PROGRESS(p)" "BLOCKED(b)" "VERIFY(v)" "|" "DONE(d)" "CANCELED(c)")))

(require 'package)
(add-to-list 'package-archives '("melpa" . "https://melpa.org/packages/") t)

(require 'auto-complete-c-headers)
