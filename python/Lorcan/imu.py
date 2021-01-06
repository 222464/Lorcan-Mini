import numpy as np
import board
import busio
import adafruit_bno055

class IMU:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        self.linear_acceleration = [ 0.0, 0.0, 0.0 ]
        self.gyro = [ 0.0, 0.0, 0.0 ]

    def poll(self):
        laccel = self.sensor.linear_acceleration
        gyro = self.sensor.gyro

        for i in range(3):
            if laccel[i] is not None:
                self.linear_acceleration[i] = laccel[i]

            if gyro[i] is not None:
                self.gyro[i] = gyro[i]

    def getLinearAccel(self):
        return self.linear_acceleration

    def getGyro(self):
        return self.gyro

