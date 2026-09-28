import scara_motion_controller_api as smc_api

import serial
import time


scara = smc_api.scara_motion_controller(port="COM4")
# Define physical dimensions of robot
link_1 = 0.125
link_2 = 0.1

# Define the limitations to the Z coordinate
z_min = 0.095
z_max = 0.15
# Settling time is the time (seconds) the robot takes after each command to correct for over/undershoot
settling_time = 1

# Set the PID tuning values for closed loop control of axis 0 and 1
P_0 = 1.3
I_0 = 0.01
D_0 = 0.001
P_1 = 1.3
I_1 = 0.01
D_1 = 0.001

# Initialise the SCARA using defined values
scara.SCARA_INITIALISE(link_1,link_2,z_min,z_max,settling_time,P_0,I_0,D_0,P_1,I_1,D_1)

# Move the SCARA joints to their neutral positions
scara.SCARA_MOVE_JOINT(2,0,1)
scara.SCARA_MOVE_JOINT(3,-60,1)
scara.SCARA_MOVE_JOINT(0,0,1)
scara.SCARA_MOVE_JOINT(1,90,1)

# Calibrate the servos for axis 0 and 1
# scara.SCARA_AUTO_CALIBRATE()

# Your Code Goes Here