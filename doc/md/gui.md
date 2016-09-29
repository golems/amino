Viewer GUI {#viewer}
====================

Amino includes a simple 3D visualization and user interface.

<img style="float:right;" src="aminogl.png" alt="GL Window">

Interface
=========


Mouse Camera Control
--------------------
| Mouse                    |  Action                         |
|--------------------------| ------------------------------- |
| Mouse Wheel              | Zoom In/Out (Camera Z)          |
| Left-Click & Drag        | Rotate World Z / Camera X       |
| Right-Click & Drag       | Translate Camera X / Y          |
| Ctrl & Mouse Wheel       | Translate Left/Right (Camera X) |
| Shift & Mouse Wheel      | Translate Up/Down (Camera Y)    |
| Ctrl Shift & Mouse Wheel | Rotate Camera Z                 |
| Left-Click & Mouse Wheel | Rotate Camera Z                 |
| Ctrl Alt & Mouse Wheel   | Rotate Global Z                 |
| Alt Shift & Mouse Wheel  | Rotate Camera X                 |


Keyboard Camera Control
-----------------------

| Key      | Action              |
|----------| ------------------- |
| Numpad - | Zoom Out (Camera Z) |
| Numpad + | Zoom In (Camera Z)  |
| Numpad 4 | Rotate World +Z     |
| Numpad 6 | Rotate World -Z     |
| Numpad 8 | Rotate Camera +X    |
| Numpad 2 | Rotate Camera -X    |
| HOME     | Reset Camera        |



Other
-----

| Key  | Action            |
|------| ---------------------------- |
| F11  | Toggle Fullscreen            |
| c    | Copy Camera Pose to clipboard (see [struct aa_tf_qv](@ref aa_tf_qv)) |
| p    | Raytrace the current frame (only when run in the @ref scenecompiler) |


3D Mouse Support
----------------

Support is provided for 3D mice from
[3DConnexion](http://www.3dconnexion.com/) .

| Enable Key  | Axis Action       |
|-------------| ------------------|
| f           | Fly Camera        |
| x           | Workspace Control |

@sa aa_rx_win_set_sg_sub()

See Also
========
* [scene_win.h](@ref scene_win.h)
* [scene_sdl.h](@ref scene_sdl.h)
* [scene_gl.h](@ref scene_gl.h)
