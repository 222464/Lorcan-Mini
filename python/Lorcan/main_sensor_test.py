import numpy as np
from imu import IMU
import time

def main():
    imu = IMU()

    frametime = 1.0 / 60.0

    print("Ready.")

    while True:
        try:
            s = time.time()

            imu.poll()

            print(imu.getLinearAccel())

            time.sleep(max(0, frametime - (time.time() - s)))
        except Exception as e:
            print(e)
            break

    print("-- Program at End --")

if __name__ == "__main__":
    main()
