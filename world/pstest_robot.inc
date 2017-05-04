include "pstest_lidar.inc"

define robot position
(
  # describe size of robot
  size [ 1.5 1.0 1.0 ]

  # describe origin of robot by the offset from the center of the shape
  # center of shape is at [0.75 0.5 0.5]. But, we want it to be at
  # [1.0 0.5 0.5]. So, offset is [-0.25 0 0]
  origin [-0.25 0 0 0]

  # describe shape of robot
  block
  (
    points 5
    point[4] [0 0]
    point[3] [1 0]
    point[2] [1.5 0.5]
    point[1] [1 1]
    point[0] [0 1]
    z [0 1]
  )

  drive "diff"

  # sick_lasers()
  topurg(pose [ 1.50 0.000 -0.500 0.000 ])

)