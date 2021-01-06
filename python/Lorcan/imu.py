import numpy as np
import board
import busio
import adafruit_bno055

class IMU:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def getLinearAccel(self):
        return self.sensor.linear_acceleration

    def getGyro(self):
        return self.sensor.gyro

