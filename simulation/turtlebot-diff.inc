include "sick.inc"

define turtlebot position (
  color "blue"      # Default color.
  drive "diff"
  gui_nose 0                    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1             # Can hit things.
  ranger_return 1               # reflects sonar beams
  blob_return 0                 # Seen by blobfinders  
  fiducial_return 0             # Seen as "1" fiducial finders

  # localization "gps"            
  # localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  localization "odom"
  odom_error [0.0 0.0 0.00 0.0]

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]       
  velocity_bounds [-1.0 1.0 -0.0 0.0 0 0 -90.0 90.0 ]          
  acceleration_bounds [-0.5 0.5 -0.0 0.0 0 0 -90 90.0 ]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]

  fancysicklaser ( 
    pose [ 0.0 0.0 0.0 0.0 ] 
    samples 100
  )
)