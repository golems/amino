;; Emacs mode for robray scene files

(require 'cc-mode)

(defvar robray-mode-hook nil)

(defvar robray-mode-map
  (let ((map (make-keymap)))
    (define-key map "\C-j" 'newline-and-indent)
    map)
  "Keymap for WPDL major mode")


(add-to-list 'auto-mode-alist '("\\.robray\\'" . robray-mode))

(defvar robray-font-lock-keywords)

(setq robray-font-lock-keywords
      (list
       '("[ \t]*\\(frame\\)\\>[ \t]*\\(\\sw+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-variable-name-face nil t))
       '("[ \t]*\\(parent\\)\\>[ \t]*\\(\\sw+\\)?"
         (1 font-lock-type-face) (2 font-lock-variable-name-face nil t))
       '("[ \t]*\\(class\\)\\>[ \t]*\\(\\sw+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-type-face nil t))
       '("[ \t]*\\(isa\\)\\>[ \t]*\\(\\sw+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-type-face nil t))
       (cons (regexp-opt '("geometry" "isa") 'words)
             'font-lock-keyword-face)
       (cons (regexp-opt '("translation" "quaternion" "shape" "color" "alpha"
                           "dimension" "radius" "height" "start_radius" "end_radius") 'words)
             'font-lock-type-face)
       (cons (regexp-opt '("mesh" "box" "cone" "cylinder" "sphere") 'words)
             'font-lock-constant-face)))


(define-derived-mode robray-mode c-mode
  "robray mode"
  "Major mode for editing Robray Scene files"
  (setq font-lock-defaults '(robray-font-lock-keywords nil nil ((?_ . "w")))))

(modify-syntax-entry ?# "< b" robray-mode-syntax-table)

(provide 'robray-mode)
