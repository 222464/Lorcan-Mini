import numpy as np
from ik import DeltaIK
from gamepad import Gamepad
from motors import Motors
from config import *
from manual_controller import ManualController
import time
import math

MODE_DEFAULT = 0
MODE_READY = 1
MODE_EXTEND = 2
MODE_RECOVER = 3

def main():
    gamepad = Gamepad()
    motors = Motors()

    iks = []

    for i in range(4):
        iks.append(DeltaIK())

    controller = ManualController(iks)

    angles = 8 * [ 0.0 ]
    kPs = 8 * [ 1.0 ]

    frametime = 1.0 / 60.0

    highKp = 1.0
    lowKp = 0.4

    # Jump
    ready_time = 1.0
    extend_time = 0.4
    recover_time = 1.0
    jump_timer = 0.0
    jump_dir = 0.0
    jump_height = -controller.walkHeight
    jump_offset = 25.0

    mode = MODE_DEFAULT

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

            pad_x = min(1.0, max(-1.0, gamepad.get_left_thumbstick_x()))
            pad_y = min(1.0, max(-1.0, gamepad.get_left_thumbstick_y()))

            dist = math.sqrt(pad_x * pad_x + pad_y * pad_y)

            if abs(pad_y) < 0.02:
                pad_y = 0.0

            kPs = 8 * [ highKp ]

            if mode == MODE_DEFAULT:
                controller.step(dist if pad_y > 0.0 else -dist, -pad_x, frametime)

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

                if gamepad.is_b_down():
                    mode = MODE_READY

                    jump_timer = 0.0
            elif mode == MODE_READY:
                jump_dir = -pad_y

                for i in range(4):
                    iks[i].update(np.array([ jump_dir * jump_offset * 0.2, jump_height * 0.6 ]))

                jump_timer += frametime

                if jump_timer >= ready_time and not gamepad.is_b_down():
                    mode = MODE_EXTEND

                    jump_timer = 0.0
            elif mode == MODE_EXTEND:
                for i in range(4):
                    iks[i].update(np.array([ -jump_dir * jump_offset * 3.0, jump_height * 1.4 ]))

                jump_timer += frametime

                if jump_timer >= extend_time:
                    mode = MODE_RECOVER

                    jump_timer = 0.0

            elif mode == MODE_RECOVER:
                for i in range(4):
                    iks[i].update(np.array([ 0.0, jump_height ]))

                jump_timer += frametime

                if jump_timer >= recover_time:
                    mode = MODE_DEFAULT

                    jump_timer = 0.0

                    controller.reset()

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
