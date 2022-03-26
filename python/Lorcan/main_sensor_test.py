import numpy as np
from imu import IMU
from flow import Flow
import time

def main():
    imu = IMU()
    flow = Flow()

    frametime = 1.0 / 60.0

    print("Ready.")

    while True:
        try:
            s = time.time()

            imu.poll()

            print(imu.getLinearAccel())
            print(flow.get_flow())

            time.sleep(max(0, frametime - (time.time() - s)))
        except Exception as e:
            print(e)
            break

    print("-- Program at End --")

if __name__ == "__main__":
    main()
