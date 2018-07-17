# map.inc - useful setup for a floorplan bitmap
# Authors: Richard Vaughan
# $Id$

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"
  
  # most maps will need a bounding box
  boundary 0

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 1
  gripper_return 0
  fiducial_return 0
  ranger_return 1

  obstacle_return 1
)

define zone model
(
  color	"orange"
  size [ 100 100 0.08 ]

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0

  # insensible to collision and range sensors
  obstacle_return 1
  ranger_return -1  # transparent to range sensors
)


