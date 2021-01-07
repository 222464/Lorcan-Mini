import serial
import numpy as np
import time

class Motors:
    def __init__(self, port="/dev/ttyACM0"):
        self.ser = serial.Serial(port, 9600)
        
        if self.ser.in_waiting:
            self.ser.readall()

        self.angles = np.zeros(8)

    def sendCommands(self, angles, kPs):
        lvalues = []

        for i in range(8):
            lvalues += [ angles[i] / np.pi * 0.5 + 0.5, kPs[i] ]
        
        values = np.array(lvalues)

        ivalues = (values * 255.0 + 0.5).astype(np.int)

        bvalues = bytearray(ivalues.tolist())

        self.ser.write(bvalues)

    def readAngles(self):
        while self.ser.in_waiting < 8:
            time.sleep(0.001)

        bangles = bytes()

        while self.ser.in_waiting >= 8:
            bangles = self.ser.read(8)

        self.angles = (np.array(list(bangles)) / 255.0 * 2.0 - 1.0) * np.pi

        return self.angles

    def __del__(self):
        self.sendCommands(8 * [ 0.0 ], 8 * [ 0.1 ])

        if self.ser:
            self.ser.close()
