# Author(s):  Anton Deguet
# Created on: 2020-11-23
#
# Copyright (c) 2020-2022 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Given a class with CRTK operating state, waits for busy/not busy conditions
#
# - wait(is_busy=True) waits for a new is_busy=True operating state to arrive
# - wait(is_busy=False) waits for *transition* to is_busy=False, i.e. waits
#       for is_busy=True followed by is_busy=False
class wait_move_handle:
    """Class for wait handle returned by move commands.

    """
    
    def __init__(self, class_instance, ral):
        self.__inst = class_instance
        self.__ral = ral
        self.__start_time = ral.now()

        if not class_instance:
            self.wait = self._unsupported
            self.is_busy = self._unsupported

    def _unsupported(self, *args, **kwargs):
        raise RuntimeWarning("can't wait, class doesn't support CRTK operating state")

    def wait(self, is_busy = False, timeout = 30.0):
        """Wait

        """
        if self.__ral.is_shutdown():
            return False

        return self.__inst.wait_for_busy(is_busy = is_busy,
                                       start_time = self.__start_time,
                                       timeout = timeout)

    def is_busy(self, timeout = 30.0):
        # if we keep asking past timeout, throw an exception
        if (self.__ral.now() - self.__start_time) > self.__ral.create_duration(timeout):
            raise RuntimeWarning('is_busy() called after timeout')

        return self.__inst.is_busy(start_time = self.__start_time)
