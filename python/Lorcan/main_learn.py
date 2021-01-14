import numpy as np
from gamepad import Gamepad
from motors import Motors
from imu import IMU
from config import *
import time
import os.path

import pyaogmaneo as pyaon

saveFileName = "lorcan_mini_rltrained.ohr"

def main():
    print("Controls:")
    print("A - shutdown")
    print("B - pause")
    print("X - load")
    print("Y - save")

    print("Initializing...")

    gamepad = Gamepad()
    motors = Motors()
    imu = IMU()

    h = pyaon.Hierarchy()

    h.initFromFile("lorcan_mini_pretrained.ohr")

    angles = 8 * [ 0.0 ]
    kPs = 8 * [ 1.0 ]
    current_angles = 8 * [ 0.0 ]

    frametime = 1.0 / 30.0

    bPressedPrev = False
    xPressedPrev = False
    yPressedPrev = False

    paused = True

    print("Ready.")

    while True:
        try:
            s = time.time()

            if not gamepad.process_events():
                print("Controller disconnected, waiting for reconnect...")

                while not gamepad.reconnect():
                    time.sleep(0.5)

                print("Controller reconnected.")

            if gamepad.is_a_down():
                print("Shutdown button (A) has been pressed, shutting down...")
                break

            bPressed = gamepad.is_b_down()

            if bPressed and not bPressedPrev:
                paused = not paused

                if paused:
                    print("Paused.")
                else:
                    print("Unpaused.")

            bPressedPrev = bPressed

            xPressed = gamepad.is_x_down()

            if xPressed and not xPressedPrev:
                if os.path.isfile(saveFileName):
                    print("Loading...")

                    h = pyaon.Hierarchy()
                    h.initFromFile(saveFileName)

                    print("Loaded.")
                else:
                    print("Save file not found.")

            xPressedPrev = xPressed

            yPressed = gamepad.is_y_down()

            if yPressed and not yPressedPrev:
                print("Saving...")

                h.saveToFile(saveFileName)

                print("Saved.")

            yPressedPrev = yPressed

            priopSDR = 8 * [ 0 ]

            for i in range(8):
                priopSDR[i] = int(min(1.0, max(0.0, (angles[i] - current_angles[i]) / maxAngleRange * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)

            imuSDR = 6 * [ 0 ]

            imu.poll()
            linearAccel = imu.getLinearAccel()
            gyro = imu.getGyro()

            # IMU
            imuSDR[0] = int(min(1.0, max(0.0, np.tanh(linearAccel[0] * linearAccelScale) * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)
            imuSDR[1] = int(min(1.0, max(0.0, np.tanh(linearAccel[1] * linearAccelScale) * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)
            imuSDR[2] = int(min(1.0, max(0.0, np.tanh(linearAccel[2] * linearAccelScale) * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)
            imuSDR[3] = int(min(1.0, max(0.0, np.tanh(gyro[0] * gyroScale) * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)
            imuSDR[4] = int(min(1.0, max(0.0, np.tanh(gyro[1] * gyroScale) * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)
            imuSDR[5] = int(min(1.0, max(0.0, np.tanh(gyro[2] * gyroScale) * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)

            reward = linearAccel[1]

            if not paused:
                h.step([ priopSDR, imuSDR, h.getPredictionCIs(2) ], True, reward, False)

                # Determine angles and kPs
                for i in range(16):
                    if i >= 8: # Kp
                        kPs[i - 8] = h.getPredictionCIs(2)[i] / float(motorRes - 1)
                    else: # Angle
                        angles[i] = (h.getPredictionCIs(2)[i] / float(motorRes - 1) * 2.0 - 1.0) * maxAngleRange

            motors.sendCommands(angles, kPs)

            current_angles = motors.readAngles()

            time.sleep(max(0, frametime - (time.time() - s)))
        except Exception as e:
            print(e)
            break

    print("-- Program at End --")

if __name__ == "__main__":
    main()
