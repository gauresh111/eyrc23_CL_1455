#!/usr/bin/env python3

import rviz2
import os

def import_scene(scene_file_path):
  """Imports a .scene file to RViz2.

  Args:
    scene_file_path: The path to the .scene file.
  """

  rvi = rviz2.Rviz()
  scene_display = rvi.add_display(rviz2.displays.SceneDisplay)
  scene_display.load_scene(os.path.abspath(scene_file_path))
  rvi.show()

if __name__ == "__main__":
  scene_file_path = "/home/gauresh/rasck.scene"
  import_scene(scene_file_path)