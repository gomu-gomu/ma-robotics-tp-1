import numpy as np
from controller import Robot, Motor



# Define the path points and orientations
xrr = np.array([0, 0.4, 0.4, 0])
yrr = np.array([0, 0, 0.4, 0.4])
thetarr = np.array([0, np.pi/2, np.pi/2, np.pi/2])

# Initialize the Webots Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())*2

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Starting position and orientation
X0 = np.array([0, 0])
ori = 0.0

# Define wheel radius and distance between wheels
wheel_radius = 0.02  # Example value, adjust as needed
wheel_distance = 0.1  # Example value, adjust as needed

for i in range(1, 4):
  theta = thetarr[i]
  X1 = np.array([xrr[i], yrr[i]])
  T = np.array([[np.cos(theta), -np.sin(theta), X1[0]], [np.sin(theta), np.cos(theta), X1[1]], [0, 0, 1]])
  x = np.transpose(T[0:2, 0:2]) @ np.array([X1[0] - X0[0], X1[1] - X0[1]])
  vx = x[0] / timestep
  vy = x[1] / timestep

  # Calculate wheel velocities
  v = np.sqrt(vx**2 + vy**2)
  omega = (theta - ori) / timestep
  left_speed = (v - (wheel_distance / 2) * omega) / wheel_radius
  right_speed = (v + (wheel_distance / 2) * omega) / wheel_radius

  # Set wheel velocities
  left_motor.setVelocity(left_speed)
  right_motor.setVelocity(right_speed)

  # Step simulation
  for _ in range(int(timestep)):
    robot.step(timestep)

  # Update position and orientation
  X0 = X1
  ori = theta

  print("x = ", x)
  print(" ")
