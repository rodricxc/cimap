include "sick.inc"

define kinect ranger
(
  sensor
  (
    range_max 1.0
    fov 360
    samples 16
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]

)

define camera_sensor camera
(
  # laser properties
  resolution [ 32 32 ]
  range [ 0.2 8.0 ]
  fov [ 70.0 40.0 ]
  pantilt [ 0.0 0.0 ]

  # model properties
  size [ 0.1 0.07 0.05 ]
  color "black"
  watts 100.0 # TODO find watts for sony pan-tilt camera
)

define turtlebot position
(
  color "orange"      # Default color.
  drive "omni"
  gui_nose 0                    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1             # Can hit things.
  ranger_return 1             # reflects sonar beams
  blob_return 0                 # Seen by blobfinders  
  fiducial_return 0             # Seen as "1" fiducial finders

  # localization "gps"            
  # localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  localization "odom"
  odom_error [0.0 0.0 0.00 0.0]

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]       
  velocity_bounds [-0.9 0.9 -0.9 0.9 0 0 -90.0 90.0 ]          
  acceleration_bounds [-0.5 0.5 -0.5 0.5 0 0 -90 90.0 ]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]

  fancysicklaser( 
    pose [ 0.0 0.0 0.0 0.0 ] 
    samples 100
  )
  
  #kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  #camera_sensor(pose [ -0.1 0.0 -0.11 1.0 ])
)