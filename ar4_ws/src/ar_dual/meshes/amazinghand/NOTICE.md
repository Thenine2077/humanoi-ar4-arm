# AmazingHand meshes

`ah_right_dark.stl` and `ah_right_light.stl` are derived from the AmazingHand right-hand model by
Pollen Robotics (https://github.com/pollen-robotics/AmazingHand,
`Demo/AHSimulation/AHSimulation/AH_Right/mjcf`), licensed under the Apache License 2.0.

Changes: all visual parts were placed at the model's reference (CAD) pose and merged into two meshes
by colour (dark = servos and hand plate, light = printed frames and finger shells). Screws, pins,
washers, rods, spacers and bushings were left out. Vertices were snapped to a 1.2 mm grid to cut the
size from ~440k to ~62k triangles. Frame: Z points from the wrist interface face (z = 0) toward the
fingertips, +X is the palm side.

Regenerate: `python -I ar_dual/scripts/mjcf_flatten.py <AH_Right/mjcf dir> <out dir>` (env CELL = grid in m).

`ah_left_dark.stl` and `ah_left_light.stl` are the two right-hand meshes mirrored across the y = 0
plane (triangle order reversed so the normals still face out), standing in for the left hand on
the right arm. Same frame and licence.
