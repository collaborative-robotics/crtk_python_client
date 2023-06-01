# Author(s):  Anton Deguet
# Created on: 2023-05-30
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import threading
import rospy
import sensor_msgs.msg

class joystick_button:
    def __init__(self, button_name, index = 0):
        self.__button_name = button_name
        self.__index = index
        self.__value = None
        self.__user_callback = None
        self.__event = threading.Event()
        self.__subscriber = rospy.Subscriber(button_name,
                                             sensor_msgs.msg.Joy, self.__callback,
                                             queue_size = 10)

    def __del__(self):
        self.__subscriber.unregister()

    def __callback(self, event):
        self.__value = event.buttons[self.__index]
        self.__event.set()
        if not self.__user_callback is None:
            self.__user_callback(self.__value)

    def set_callback(self, callback = None):
        self.__user_callback = callback

    def value(self):
        return self.__value

    def wait(self, timeout = None, value = None):
        ready = False
        while not ready:
            self.__event.clear()
            if not self.__event.wait(timeout):
                return False
            else:
                if value == None or value == self.__value:
                    ready = True
        return False
