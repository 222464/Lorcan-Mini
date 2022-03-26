import numpy as np
from gamepad import Gamepad
from motors import Motors
from imu import IMU
from flow import Flow
from config import *
import time
import os.path

import pyaogmaneo as neo

saveFileName = "lorcan_mini_rltrained.ohr"

def main():
    print("Controls:")
    print("A - shutdown")
    print("B - pause")
    print("X - save")
    print("Y - load")

    print("Initializing...")

    gamepad = Gamepad()
    motors = Motors()
    imu = IMU()
    flow = Flow()

    neo.setNumThreads(3)

    h = neo.Hierarchy()

    h.initFromFile("lorcan_mini_pretrained.ohr")

    angles = 8 * [ 0.0 ]
    kPs = 8 * [ 0.1 ]
    current_angles = 8 * [ 0.0 ]

    actionsExp = h.getPredictionCIs(2)

    frametime = 1.0 / 60.0

    paused = True

    epsilon = 0.0

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

            if gamepad.is_b_clicked():
                paused = not paused

                if paused:
                    print("Paused.")
                else:
                    print("Unpaused.")

            if gamepad.is_y_clicked():
                if os.path.isfile(saveFileName):
                    print("Loading...")

                    h = neo.Hierarchy()
                    h.initFromFile(saveFileName)

                    print("Loaded.")
                else:
                    print("Save file not found.")

            if gamepad.is_x_clicked():
                print("Saving...")

                h.saveToFile(saveFileName)

                print("Saved.")

            priopSDR = 8 * [ 0 ]

            for i in range(8):
                priopSDR[i] = int(min(1.0, max(0.0, (angles[i] - current_angles[i]) / maxAngleRange * 0.5 + 0.5)) * (sensorRes - 1) + 0.5)

            imuSDR = 6 * [ sensorRes // 2 ]

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

            forward_flow = flow.get_flow()[1]
            reward = forward_flow * 0.1

            if gamepad.is_r_down():
                print(reward)

            if not paused:
                h.step([ priopSDR, imuSDR, actionsExp ], True, reward)

                actionsExp = h.getPredictionCIs(2)

                #for i in range(len(actionsExp)):
                #    if np.random.rand() < epsilon:
                #        actionsExp[i] = np.random.randint(0, motorRes)

                # Determine angles and kPs
                for i in range(16):
                    if i >= 8: # Kp
                        kPs[i - 8] = actionsExp[i] / float(motorRes - 1)
                    else: # Angle
                        angles[i] = (actionsExp[i] / float(motorRes - 1) * 2.0 - 1.0) * maxAngleRange

            motors.sendCommands(angles, kPs)

            current_angles = motors.readAngles()

            # Wrap current angles
            for i in range(len(current_angles)):
                if current_angles[i] > np.pi:
                    current_angles[i] -= np.pi

            time.sleep(max(0, frametime - (time.time() - s)))
        except Exception as e:
            print(e)
            break

    print("-- Program at End --")

if __name__ == "__main__":
    main()
