import numpy as np
from ik import DeltaIK
from copy import copy

def stepFunc(x):
    sx = np.sin(x * np.pi)
    return np.square(sx)

class ManualController:
    def __init__(self, iks):
        self.iks = iks

        self.targetPositions = []
        self.resetFromPositions = []
        self.resetToPositions = []

        self.walkHeight = 60.0

        for _ in range(4):
            self.targetPositions.append(np.array([ 0.0, -self.walkHeight ]))
            self.resetFromPositions.append(np.array([ 0.0, -self.walkHeight ]))
            self.resetToPositions.append(np.array([ 0.0, -self.walkHeight ]))
        
        self.stepTimer = 0.0
        self.stepTime = 0.2

        self.recenterStep = 0
        self.recenterSteps = 2

        self.speed = 90.0

        self.stepLenTimer0 = 0.0
        self.stepLenTimer1 = 0.0
        self.stepLenTime = 0.07

        self.stepHeight = 23.0

        self.minWalkRate = 0.1

        self.resetDecay = 0.7

        self.stepSide = 0
        self.stepping0 = False
        self.stepping1 = False

        self.walkRatePrev = 0.0

    def step(self, walkRate, turnRate, dt):
        if self.stepTimer > self.stepTime:
            self.stepTimer = 0.0

            if self.stepSide == 0:
                self.stepSide = 1
                self.stepping0 = False
                self.stepping1 = True
                self.stepLenTimer1 = 0.0
                self.resetFromPositions[2] = copy(self.targetPositions[2])
                self.resetFromPositions[1] = copy(self.targetPositions[1])

                self.resetToPositions[2] = copy(self.targetPositions[2])
                self.resetToPositions[1] = copy(self.targetPositions[1])

                self.resetToPositions[2][0] *= -self.resetDecay
                self.resetToPositions[1][0] *= -self.resetDecay
            else:
                self.stepSide = 0
                self.stepping0 = True
                self.stepping1 = False
                self.stepLenTimer0 = 0.0
                self.resetFromPositions[0] = copy(self.targetPositions[0])
                self.resetFromPositions[3] = copy(self.targetPositions[3])

                self.resetToPositions[0] = copy(self.targetPositions[0])
                self.resetToPositions[3] = copy(self.targetPositions[3])

                self.resetToPositions[0][0] *= -self.resetDecay
                self.resetToPositions[3][0] *= -self.resetDecay

        if self.stepping0:
            self.stepLenTimer0 += dt

            if self.stepLenTimer0 > self.stepLenTime:
                self.stepping0 = False
                self.stepLenTimer0 = self.stepLenTime

                if self.recenterStep > 0:
                    self.recenterStep -= 1

            self.targetPositions[0][0] = self.resetFromPositions[0][0] + (self.resetToPositions[0][0] - self.resetFromPositions[0][0]) * (self.stepLenTimer0 / self.stepLenTime)
            self.targetPositions[0][1] = stepFunc(self.stepLenTimer0 / self.stepLenTime) * self.stepHeight - self.walkHeight

            self.targetPositions[3][0] = self.resetFromPositions[3][0] + (self.resetToPositions[3][0] - self.resetFromPositions[3][0]) * (self.stepLenTimer0 / self.stepLenTime)
            self.targetPositions[3][1] = stepFunc(self.stepLenTimer0 / self.stepLenTime) * self.stepHeight - self.walkHeight
        else:
            self.targetPositions[0][0] += -walkRate * dt * self.speed * min(1.0, 1.0 - turnRate * 2.0)
            self.targetPositions[0][1] = -self.walkHeight

            self.targetPositions[3][0] += -walkRate * dt * self.speed * min(1.0, 1.0 + turnRate * 2.0)
            self.targetPositions[3][1] = -self.walkHeight

        if self.stepping1:
            self.stepLenTimer1 += dt

            if self.stepLenTimer1 > self.stepLenTime:
                self.stepping1 = False
                self.stepLenTimer1 = self.stepLenTime

                if self.recenterStep > 0:
                    self.recenterStep -= 1

            self.targetPositions[2][0] = self.resetFromPositions[2][0] + (self.resetToPositions[2][0] - self.resetFromPositions[2][0]) * (self.stepLenTimer1 / self.stepLenTime)
            self.targetPositions[2][1] = stepFunc(self.stepLenTimer1 / self.stepLenTime) * self.stepHeight - self.walkHeight

            self.targetPositions[1][0] = self.resetFromPositions[1][0] + (self.resetToPositions[1][0] - self.resetFromPositions[1][0]) * (self.stepLenTimer1 / self.stepLenTime)
            self.targetPositions[1][1] = stepFunc(self.stepLenTimer1 / self.stepLenTime) * self.stepHeight - self.walkHeight
        else:
            self.targetPositions[2][0] += -walkRate * dt * self.speed * min(1.0, 1.0 + turnRate * 2.0)
            self.targetPositions[2][1] = -self.walkHeight

            self.targetPositions[1][0] += -walkRate * dt * self.speed * min(1.0, 1.0 - turnRate * 2.0)
            self.targetPositions[1][1] = -self.walkHeight

        for i in range(4):
            self.iks[i].update(self.targetPositions[i])

        if np.absolute(walkRate) > self.minWalkRate:
            self.stepTimer += dt
        else:
            if np.absolute(self.walkRatePrev) > self.minWalkRate:
                self.recenterStep = self.recenterSteps

            if self.recenterStep > 0:
                self.stepTimer += dt
            else:
                self.stepTimer = 0.0
                self.stepLenTimer0 = 0.0
                self.stepLenTimer1 = 0.0

        self.walkRatePrev = walkRate

    def reset(self):
        self.stepTimer = 0.0
        self.stepLenTimer0 = 0.0
        self.stepLenTimer1 = 0.0
        self.walkRatePrev = 0.0

        for i in range(4):
            self.targetPositions[i] = np.array([ 0.0, -self.walkHeight ])

