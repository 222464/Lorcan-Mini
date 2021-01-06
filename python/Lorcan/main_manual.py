import numpy as np
from ik import DeltaIK
from gamepad import Gamepad
from motors import Motors
from config import *
from manual_controller import ManualController
import time

def main():
    gamepad = Gamepad()
    motors = Motors()

    iks = []

    for i in range(4):
        iks.append(DeltaIK())

    controller = ManualController(iks)

    angles = 8 * [ 0.0 ]
    kPs = 8 * [ 0.1 ]

    frametime = 1.0 / 60.0

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

            pad_y = gamepad.get_left_thumbstick_y()

            controller.step(pad_y, frametime)

            # Map IK leg results to motors
            for i in range(4):
                angles[legMap[i][0]] = legDirs[i][0] * (iks[i].angle - (np.pi / 2.0 - iks[i].A))
                angles[legMap[i][1]] = legDirs[i][1] * (iks[i].angle + (np.pi / 2.0 - iks[i].A))

            motors.sendCommands(angles, kPs)

            time.sleep(max(0, frametime - (time.time() - s)))
        except Exception as e:
            print(e)
            break

    print("-- Program at End --")

if __name__ == "__main__":
    main()
