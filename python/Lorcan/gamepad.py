from evdev import InputDevice, categorize, ecodes
import socket
import errno
from collections import deque

class Gamepad:
    def __init__(self):
        self.device = InputDevice("/dev/input/event0")
        self.event_queue = deque()
        self.a_button = False
        self.b_button = False
        self.x_button = False
        self.y_button = False
        self.r_button = False
        self.prev_a_button = False
        self.prev_b_button = False
        self.prev_x_button = False
        self.prev_y_button = False
        self.prev_r_button = False
        self.a_clicked = False
        self.b_clicked = False
        self.x_clicked = False
        self.y_clicked = False
        self.r_clicked = False
        self.thumbstick_max = 255.0
        self.thumbstick_middle = self.thumbstick_max / 2.0
        self.left_thumbstick_x = 0.0
        self.left_thumbstick_y = 0.0
        self.right_thumbstick_x = 0.0
        self.right_thumbstick_y = 0.0

    # Returns False if failed to reconnect
    def reconnect(self):
        try:
            self.device = InputDevice("/dev/input/event0")
        except:
            return False
        
        self.a_button = False
        self.b_button = False
        self.x_button = False
        self.y_button = False
        self.r_button = False
        self.prev_a_button = False
        self.prev_b_button = False
        self.prev_x_button = False
        self.prev_y_button = False
        self.prev_r_button = False
        self.a_clicked = False
        self.b_clicked = False
        self.x_clicked = False
        self.y_clicked = False
        self.r_clicked = False
        self.thumbstick_max = 255.0
        self.thumbstick_middle = self.thumbstick_max / 2.0
        self.left_thumbstick_x = 0.0
        self.left_thumbstick_y = 0.0
        self.right_thumbstick_x = 0.0
        self.right_thumbstick_y = 0.0

        return True
    
    # Returns False if failed to get events (controller disconnected)
    def process_events(self):
        self.a_button = False
        self.b_button = False
        self.x_button = False
        self.y_button = False
        self.r_button = False
        self.a_clicked = False
        self.b_clicked = False
        self.x_clicked = False
        self.y_clicked = False
        self.r_clicked = False

        try:
            for event in self.device.read():
                self.event_queue.append(event)
        except socket.error as e:
            err = e.args[0]
            if err != errno.EAGAIN and err != errno.EWOULDBLOCK:
                return False

        while len(self.event_queue) > 0:
            event = self.event_queue.popleft()
            # Buttons
            if event.type == ecodes.EV_KEY:
                state = event.value == 1
                if event.code == 304:
                    self.a_button = state
                elif event.code == 305:
                    self.b_button = state
                elif event.code == 307:
                    self.x_button = state
                elif event.code == 308:
                    self.y_button = state
                elif event.code == 311:
                    self.r_button = state
                # This break is needed to allow the
                # function caller to process the changed state
                # before it's potentially changed twice or more
                # during this loop (e.g. button down, button up,
                # resulting in the user only getting a buttom up).
                break
            # Analog
            elif event.type == ecodes.EV_ABS:
                absolute_event = categorize(event)
                typecode = ecodes.bytype[absolute_event.event.type][absolute_event.event.code]
                value = (absolute_event.event.value - self.thumbstick_middle) / self.thumbstick_middle
                if typecode == "ABS_X":
                    self.left_thumbstick_x = value
                elif typecode == "ABS_Y":
                    self.left_thumbstick_y = -value

        if self.prev_a_button and not self.a_button:
            self.a_clicked = True
        if self.prev_b_button and not self.b_button:
            self.b_clicked = True
        if self.prev_x_button and not self.x_button:
            self.x_clicked = True
        if self.prev_y_button and not self.y_button:
            self.y_clicked = True
        if self.prev_r_button and not self.r_button:
            self.r_clicked = True

        self.prev_a_button = self.a_button
        self.prev_b_button = self.b_button
        self.prev_x_button = self.x_button
        self.prev_y_button = self.y_button
        self.prev_r_button = self.r_button

        return True

    def is_a_down(self):
        return self.a_button

    def is_b_down(self):
        return self.b_button
    
    def is_x_down(self):
        return self.x_button
    
    def is_y_down(self):
        return self.y_button

    def is_r_down(self):
        return self.r_button

    def is_a_clicked(self):
        return self.a_clicked

    def is_b_clicked(self):
        return self.b_clicked
    
    def is_x_clicked(self):
        return self.x_clicked
    
    def is_y_clicked(self):
        return self.y_clicked

    def is_r_clicked(self):
        return self.r_clicked

    # Returns -1.0 to 1.0
    def get_left_thumbstick_x(self):
        return self.left_thumbstick_x

    # Returns -1.0 to 1.0
    def get_left_thumbstick_y(self):
        return self.left_thumbstick_y
    
    def print(self):
        print(self.device)

