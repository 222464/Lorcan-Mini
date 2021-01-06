import numpy as np

class DeltaIK:
    def __init__(self):
        self.reset()

    def reset(self):
        self.a = 62.0
        self.b = 36.0

        self.p_origin = np.array([ 0.0, 0.0 ]) # Optional: offset to better hold CG

        self.angle = 0.0

        # Angles to compute
        self.A = 0.0
        self.B = 0.0
        self.C = 0.0

        self.c = 50.0

        self.p_end = np.array([ 0.0, -self.c ])

    def update(self, target_pos):
        # IK

        # Find angle to target pos
        d_target = target_pos - self.p_origin

        norm = np.sqrt(d_target[0] * d_target[0] + d_target[1] * d_target[1])

        self.angle = np.arctan2(d_target[1], d_target[0]) + np.pi * 0.5

        self.c = np.minimum(self.a + self.b, norm)

        # Determine angles
        self.A = np.arccos(np.clip((self.b * self.b + self.c * self.c - self.a * self.a) / (2.0 * self.b * self.c), -1.0, 1.0))
        self.B = np.arccos(np.clip((self.a * self.a + self.c * self.c - self.b * self.b) / (2.0 * self.a * self.c), -1.0, 1.0))
        self.C = np.arccos(np.clip((self.a * self.a + self.b * self.b - self.c * self.c) / (2.0 * self.a * self.b), -1.0, 1.0))

        self.p_end = self.p_origin + d_target * self.c / np.maximum(0.0001, norm)
