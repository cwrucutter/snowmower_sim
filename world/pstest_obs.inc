define pentagon_sm model
(
  size [1.9022 1.8090 1.0000]

  # describe shape of robot
  block
  (
    points 5
    point[4] [ 0.0000  1.0000]
    point[3] [-0.9511  0.3090]
    point[2] [-0.5878 -0.8090]
    point[1] [ 0.5878 -0.8090]
    point[0] [ 0.9511  0.3090]
    z [0 1]
  )

  gui_nose 1
  ranger_return 1
)
