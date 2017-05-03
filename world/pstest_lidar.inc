define sick_laser sensor
(
  size [ 0.05 0.05 0.10 ]
  range [ 0.0 30.0 ]
  fov 180
  samples 181
)

define sick_lasers ranger
(
  sick_laser( pose [ 0.0 0.0 0.0 0.0 ] )
)

define topurg ranger
(
  sensor( 			
    range [ 0.0  30.0 ]
    fov 180
   samples 181
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)
