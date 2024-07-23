(require 'package)
(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(custom-enabled-themes '(adwaita))
 '(custom-safe-themes
   '("d92c1c36a5181cf629749bf6feee1886cf6bce248ab075c9d1b1f6096fea9539" "603a831e0f2e466480cdc633ba37a0b1ae3c3e9a4e90183833bc4def3421a961" "1711947b59ea934e396f616b81f8be8ab98e7d57ecab649a97632339db3a3d19" default))
 '(display-time-mode t)
 '(ede-project-directories '("/home/boo/Workspace/thesis"))
 '(inhibit-startup-screen t)
 '(package-selected-packages
   '(timu-spacegrey-theme dracula-theme go-complete go-autocomplete auto-complete-c-headers cmake-mode magit afternoon-theme projectile haskell-mode rust-mode))
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

(require 'auto-complete-c-headers)
(require 'package)
(add-to-list 'package-archives '("melpa" . "https://melpa.org/packages/") t)

(setq-default c-basic-offset 2)

(add-to-list 'load-path "/home/boo/.emacs.d/neotree")
(require 'neotree)
(global-set-key [f8] 'neotree-toggle)
(global-set-key (kbd "C-x t") 'ff-find-other-file)
(global-set-key (kbd "s-<up>") 'enlarge-window-horizontally)
(global-set-key (kbd "s-<down>") 'shrink-window-horizontally)

(require 'org)
(setq org-todo-keywords '((sequence "TODO(t)" "PROGRESS(p)" "BLOCKED(b)" "VERIFY(v)" "|" "DONE(d)" "CANCELED(c)")))

(put 'narrow-to-region 'disabled nil)
(add-hook 'before-save-hook 'time-stamp)
