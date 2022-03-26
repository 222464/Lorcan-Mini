import numpy as np
from ik import DeltaIK
import serial
from motors import Motors
from manual_controller import ManualController
from config import *
import time

import pyaogmaneo as neo

def main():
    neo.setNumThreads(3)

    lds = []

    for _ in range(1):
        ld = neo.LayerDesc()

        ld.hiddenSize = (5, 5, 16)

        ld.ffRadius = 2
        ld.fbRadius = 2
        ld.rRadius = 2
        
        lds.append(ld)

    h = neo.Hierarchy()

    h.initRandom([
        neo.IODesc((2, 4, sensorRes), neo.prediction, 2, 2, 32), # Priop
        neo.IODesc((2, 3, sensorRes), neo.prediction, 1, 2, 32), # IMU
        neo.IODesc((4, 4, motorRes), neo.action, 2, 2, 32) # Motor control
        ], lds)

    iks = []

    for i in range(4):
        iks.append(DeltaIK())

    controller = ManualController(iks)

    angles = 8 * [ 0.0 ]
    kPs = 8 * [ 1.0 ]
    
    highKp = 1.0
    lowKp = 0.5

    frametime = 1.0 / 60.0

    print("Ready.")

    saveTimer = 0
    errorTimer = 0
    averageError = 0.0

    while True:
        try:
            #s = time.time()

            pad_y = 1.0

            controller.step(pad_y, frametime)

            kPs = 8 * [ highKp ]

            if controller.stepping0:
                kPs[legMap[0][0]] = lowKp
                kPs[legMap[0][1]] = lowKp

                kPs[legMap[2][0]] = lowKp
                kPs[legMap[2][1]] = lowKp

            if controller.stepping1:
                kPs[legMap[1][0]] = lowKp
                kPs[legMap[1][1]] = lowKp

                kPs[legMap[3][0]] = lowKp
                kPs[legMap[3][1]] = lowKp

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
            if errorTimer >= 10:
                errorTimer = 0

                print("Error: " + str(averageError))

            errorTimer += 1

            # Save occasionally
            if saveTimer >= 500:
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
