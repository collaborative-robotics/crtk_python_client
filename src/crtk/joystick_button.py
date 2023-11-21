# Author(s):  Anton Deguet
# Created on: 2023-05-30
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import threading
import sensor_msgs.msg

class joystick_button:
    def __init__(self, ral, button_name, index = 0):
        self._index = index
        self._value = None
        self._user_callback = None
        self._event = threading.Event()
        self._subscriber = ral.subscriber(button_name, sensor_msgs.msg.Joy,
                                          self._callback, queue_size = 10)

    def _callback(self, event):
        self._value = event.buttons[self._index]
        self._event.set()
        if not self._user_callback is None:
            self._user_callback(self._value)

    def set_callback(self, callback = None):
        self._user_callback = callback

    def value(self):
        return self._value

    def wait(self, timeout = None, value = None):
        ready = False
        while not ready:
            self._event.clear()
            if not self._event.wait(timeout):
                return False
            else:
                if value == None or value == self._value:
                    ready = True
        return False
