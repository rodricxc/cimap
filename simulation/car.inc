# Desc: Device definitions for Activemedia robots.
# Author: Rezeck
# Date: 2 Nov 2016

define car_base position 
(
  color "orange"			# Default color. 
  drive "car"
  gui_nose 0                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  ranger_return 1            	# reflects sonar beams
  blob_return 0               	# Seen by blobfinders  
  fiducial_return 0           	# Seen as "1" fiducial finders

  localization "gps"         	  
  localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]				
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]					
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)

define car car_base
(
 # carro parte baixa
  block(
    points 4
    point[0] [0.500000  0.250000]
    point[1] [-0.500000  0.250000]
    point[2] [-0.500000 -0.250000]
    point[3] [0.500000 -0.250000]
    z [ 0.08 0.20]
    color "red"
  )

  # carro cima
  block
  (
    points 4
    point[0] [0.250000 0.250000]
    point[1] [0.250000 -0.250000]
    point[2] [-0.250000 -0.250000]
    point[3] [-0.250000 0.250000]
    z [ 0.2 0.3 ]
    color "red"
  )

  # farol direito
  block(
    points 4
    point[0] [0.545000 0.185000]
    point[1] [0.495000 0.185000]
    point[2] [0.495000 0.115000]
    point[3] [0.545000 0.115000]
    z [ 0.10 0.15]
    color "yellow"
  )

  # farol esq
  block(
    points 4
    point[0] [0.545000 -0.115000]
    point[1] [0.495000 -0.115000]
    point[2] [0.495000 -0.185000]
    point[3] [0.545000 -0.185000]
    z [ 0.10 0.15]
    color "yellow"
  )

  # roda frente direita
  block(
    points 4
    point[0] [0.350000 0.300000]
    point[1] [0.150000 0.300000]
    point[2] [0.150000 0.200000]
    point[3] [0.350000 0.200000]
    z [ 0.0 0.10]
    color "black"
  )

  # roda tras direita
  block(
    points 4
    point[0] [-0.150000 0.300000]
    point[1] [-0.350000 0.300000]
    point[2] [-0.350000 0.200000]
    point[3] [-0.150000 0.200000]
    z [ 0.0 0.10]
    color "black"
  )

  # roda frente esquerda
  block(
    points 4
    point[0] [0.350000 -0.200000]
    point[1] [0.150000 -0.200000]
    point[2] [0.150000 -0.300000]
    point[3] [0.350000 -0.300000]
    z [ 0.0 0.10]
    color "black"
  )

  # roda tras esquerda
  block(
    points 4
    point[0] [-0.150000 -0.200000]
    point[1] [-0.350000 -0.200000]
    point[2] [-0.350000 -0.300000]
    point[3] [-0.150000 -0.300000]
    z [ 0.0 0.10]
    color "black"
  )

  # Estimated mass in KG
  mass 23.0 

  size [0.55 0.3 0.3]
)
