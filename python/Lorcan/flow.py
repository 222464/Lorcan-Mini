from pmw3901 import PMW3901
import time
import threading

class Flow:
    def __init__(self):
        self.flow = PMW3901(spi_port=0, spi_cs=1, spi_cs_gpio=8)

        self.dx = 0
        self.dy = 0

        self._lock = threading.Lock()
        self.stop = False

        self.th = threading.Thread(target=self._get_repeat, daemon=True)
        self.th.start()

    def _get_repeat(self):
        while not self.stop:
            dx = 0
            dy = 0

            try:
                dx, dy = self.flow.get_motion()
            except Exception:
                pass

            with self._lock:
                self.dx = dx
                self.dy = dy

    def get_flow(self):
        with self._lock:
            return [ self.dx, self.dy ]

    def __del__(self):
        self.stop = True
        self.th.join()
