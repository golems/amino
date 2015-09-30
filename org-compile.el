(require 'cl)

(setq vc-follow-symlinks t
      make-backup-files nil)

(setq org-export-allow-BIND t
      org-export-html-preamble nil

      ;org-export-html-postamble "\n\nGOOGLE_ANALYTICS\n"
      org-html-postamble nil
      org-export-author-info nil
      org-export-date-info nil
      org-export-creator-info nil
      org-html-validation-link nil
      )

;; Pick up the updated org-mode in $HOME
(labels ((try-add-dir (dir)
           (when (file-exists-p dir)
             (add-to-list 'load-path dir))))
  ;(try-add-dir "~/.emacs.d")
  (try-add-dir "~/share/emacs/site-lisp/"))


(defun golems-org-emit (in out)
  (find-file in)
  (set-buffer-file-coding-system 'utf-8)
  ;; Org-mode renamed the export function.
  ;; They get demerits for breaking backwards-compatibility.
  (if (fboundp 'org-export-to-buffer)
      (org-export-to-buffer 'html  "*Org HTML Export*")
    (org-export-as-html-to-buffer 3))
  ;(list-buffers)
  (switch-to-buffer "*Org HTML Export*")
  (write-file out))

(defun ntd-org-emit-latex (in out)
  (find-file in)
  (set-buffer-file-coding-system 'utf-8)
  (org-export-as-latex-to-buffer 3)
  (list-buffers)
  (switch-to-buffer "*Org LaTeX Export*")
  (write-file out))
