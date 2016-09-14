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
       '("\\<\\(frame\\)\\>\\s-+\\([[:alnum:]][[:alnum:]_\\-]+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-variable-name-face nil t))
       '("\\<\\(parent\\)\\>\\s-+\\(\\sw+\\)?"
         (1 font-lock-type-face) (2 font-lock-variable-name-face nil t))
       '("\\<\\(class\\)\\>\\s-+\\(\\sw+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-type-face nil t))
       '("\\<\\(isa\\)\\>\\s-+\\(\\sw+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-type-face nil t))
       '("\\<\\(def\\)\\>\\s-+\\(\\sw+\\)?"
         (1 font-lock-keyword-face) (2 font-lock-constant-face nil t))
       (cons (regexp-opt '("geometry" "include" "allow_collision") 'words)
             'font-lock-keyword-face)
       (cons (regexp-opt '("translation" "quaternion" "rpy" "shape" "color" "alpha" "specular"
                           "dimension" "radius" "height" "start_radius" "end_radius" "delta" "thickness" "affords"
                           "type" "axis" "offset" "scale" "visual" "collision" "variable" "mesh") 'words)
             'font-lock-type-face)
       (cons (regexp-opt '("grid" "box" "cone" "cylinder" "sphere"
                           "fixed" "prismatic" "revolute" "π" "pi" "deg")
                         'words)
             'font-lock-constant-face)))


(define-derived-mode robray-mode c-mode
  "robray mode"
  "Major mode for editing Robray Scene files"
  (setq font-lock-defaults '(robray-font-lock-keywords nil nil ((?_ . "w")))))

(modify-syntax-entry ?# "< b" robray-mode-syntax-table)

(provide 'robray-mode)
