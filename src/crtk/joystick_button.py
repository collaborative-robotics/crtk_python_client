# Author(s):  Anton Deguet
# Created on: 2023-05-30
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import threading
import sensor_msgs.msg

class joystick_button:
    """Helper class to subscribe to a ROS joystick topic and expose a
    single button as a blocking, event-driven interface.

    Button state changes are received via the ``sensor_msgs/Joy``
    message and stored internally.  User code can either poll
    :meth:`value` or block inside :meth:`wait` until a new event
    (optionally matching a specific value) arrives.
    """

    def __init__(self, ral, button_name, index = 0):
        """Subscribe to a joystick topic and track one button.

        :param ral: RAL instance used to create the subscriber.
        :param button_name: Topic name of the ``sensor_msgs/Joy`` message.
        :type button_name: str
        :param index: Zero-based index into the ``Joy.buttons`` array
            that identifies the button of interest.  Defaults to ``0``.
        :type index: int
        """
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
        """Register a user callback to be invoked on every button event.

        The callback receives a single argument: the new button value
        (typically ``0`` or ``1`` for a digital joystick button).

        :param callback: Callable with signature ``callback(value)``.
            Pass ``None`` to remove a previously set callback.
        """
        self._user_callback = callback

    def value(self):
        """Return the most recently received button value.

        Returns ``None`` if no ``Joy`` message has been received yet.

        :return: Latest button value (usually ``0`` or ``1``), or ``None``.
        """
        return self._value

    def wait(self, timeout = None, value = None):
        """Block until a new button event is received.

        Waits for the next ``Joy`` message and optionally loops until the
        button value matches *value*.  The internal threading event is
        cleared before each wait so that only *new* events are counted.

        :param timeout: Maximum time in seconds to wait for each event.
            ``None`` (default) means wait indefinitely.
        :type timeout: float or None
        :param value: When not ``None``, keep waiting until the received
            button value equals *value* (e.g. wait for button press with
            ``value=1``).
        :return: ``True`` once a matching event is received, ``False``
            if the timeout expired before any event arrived.
        :rtype: bool
        """
        ready = False
        while not ready:
            self._event.clear()
            if not self._event.wait(timeout):
                return False
            else:
                if value == None or value == self._value:
                    ready = True
        return False
