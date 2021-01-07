import numpy as np

# Configuration of robot
legMap = [ ( 1, 0 ), ( 4, 3 ), ( 5, 6 ), ( 7, 2 ) ]
legDirs = [ ( -1, 1 ), ( 1, -1 ), ( 1, -1 ), ( -1, 1 ) ]

# Configuration of sensors
sensorRes = 31
motorRes = 11

maxAngleRange = np.pi / 4.0

linearAccelScale = 1.0
gyroScale = 1.0
