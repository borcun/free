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
 '(tool-bar-mode nil))
(package-initialize)

(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 )

(setq-default c-basic-offset 4)
(global-set-key [C-tab] 'other-window)
(global-set-key (kbd "C-x t") 'ff-find-other-file)
(global-set-key (kbd "M-<up>") 'enlarge-window-horizontally)
(global-set-key (kbd "M-<down>") 'shrink-window-horizontally)
(setq org-todo-keywords '((sequence "TODO(t)" "PROGRESSING(p)" "BLOCKED(b)" "VERIFY(v)" "|" "DONE(d)" "CANCELED(c)")))
