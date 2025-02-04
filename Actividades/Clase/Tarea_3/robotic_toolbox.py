from roboticstoolbox import DHRobot, PrismaticDH, RevoluteDH
from spatialmath import SE3
import numpy as np
 

# Define the Cartesian robot with 3 prismatic joints
robot = DHRobot(
    [
        RevoluteDH(a=0, alpha=np.pi/2, d=3),
        RevoluteDH(a=7, alpha=0,        d=0),
        RevoluteDH(a=0, alpha=-np.pi/2, d=0),
        RevoluteDH(a=0, alpha=np.pi/2, d=4),
        RevoluteDH(a=0, alpha=-np.pi/2, d=0),
        RevoluteDH(a=0, alpha=0,        d=5)
 
    ],
    base = SE3(0, 0, 0),   # shift the entire robot 0.5m along Z
    name="Fanuc"
)
 
# Print the robot's structure
print(robot)
 
# Define the displacement values for each prismatic joint
t_values = [np.radians(0), 
            np.radians(0), 
            np.radians(0), 
            np.radians(0), 
            np.radians(0), 
            np.radians(0),]  # Example: [X, Y, Z] displacements in meters
 
# Plot the robot at the specified position
robot.plot(t_values, block=True, jointaxes=True, eeframe=True, jointlabels=True)
