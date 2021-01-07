import numpy as np
from ik import DeltaIK
import serial
from motors import Motors
from manual_controller import ManualController
from config import *
import time

import pyaogmaneo as pyaon

def main():
    pyaon.setNumThreads(8)

    lds = []

    for i in range(3):
        ld = pyaon.LayerDesc()

        ld.hiddenSize = (3, 3, 16)
        
        lds.append(ld)

    h = pyaon.Hierarchy()

    h.initRandom([
        pyaon.IODesc((2, 4, sensorRes), pyaon.none, 2, 2, 2, 32), # Priop
        pyaon.IODesc((2, 3, sensorRes), pyaon.none, 2, 2, 2, 32), # IMU
        pyaon.IODesc((4, 4, motorRes), pyaon.action, 2, 2, 2, 32) # Motor control
        ], lds)

    iks = []

    for i in range(4):
        iks.append(DeltaIK())

    controller = ManualController(iks)

    angles = 8 * [ 0.0 ]
    kPs = 8 * [ 1.0 ]

    frametime = 1.0 / 30.0

    print("Ready.")

    saveTimer = 0
    errorTimer = 0
    averageError = 0.0

    while True:
        try:
            #s = time.time()

            pad_y = 1.0

            controller.step(pad_y, frametime)

            # Map IK leg results to motors
            for i in range(4):
                angles[legMap[i][0]] = legDirs[i][0] * (iks[i].angle - (np.pi / 2.0 - iks[i].A))
                angles[legMap[i][1]] = legDirs[i][1] * (iks[i].angle + (np.pi / 2.0 - iks[i].A))

            #motors.sendCommands(angles, kPs)
            priopSDR = 8 * [ 0 ]

            for i in range(8):
                priopSDR[i] = np.random.randint(0, sensorRes) # Train with noise

            imuSDR = 6 * [ 0 ]

            # Random IMU training
            for i in range(6):
                imuSDR[i] = np.random.randint(0, sensorRes)

            # Train with commands from manual controller
            motorSDR = 16 * [ 0 ]

            for i in range(16):
                if i >= 8: # Kp
                    motorSDR[i] = int(kPs[i - 8] * (motorRes - 1) + 0.5) # Train with maximum default
                else: # Angle
                    motorSDR[i] = int(min(1.0, max(0.0, angles[i] / maxAngleRange * 0.5 + 0.5)) * (motorRes - 1) + 0.5)

            error = 0.0

            for i in range(len(motorSDR)):
                delta = (motorSDR[i] - h.getPredictionCIs(2)[i]) / float(motorRes - 1)

                error += delta * delta

            averageError = 0.99 * averageError + 0.01 * error

            h.step([ priopSDR, imuSDR, motorSDR ], True, 0.0, True)

            # Show error rate occasionally
            if errorTimer >= 1000:
                errorTimer = 0

                print("Error: " + str(averageError))

            errorTimer += 1

            # Save occasionally
            if saveTimer >= 10000:
                saveTimer = 0

                print("Saving...")
                
                h.saveToFile("lorcan_mini_pretrained.ohr")

                print("Saved.")

            saveTimer += 1

            #time.sleep(max(0, frametime - (time.time() - s)))
        except Exception as e:
            print(e)
            break

    print("-- Program at End --")

if __name__ == "__main__":
    main()
