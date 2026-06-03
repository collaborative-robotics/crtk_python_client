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
        """Create a move-wait handle.

        Records the current ROS time as the *start time* so that
        subsequent :meth:`wait` and :meth:`is_busy` calls can determine
        whether busy/idle transitions occurred *after* the move command
        was issued.

        When *class_instance* is falsy (e.g. ``None``) the handle is put
        into an unsupported state where :meth:`wait` and :meth:`is_busy`
        raise a ``RuntimeWarning``.

        :param class_instance: Object that exposes CRTK operating-state
            methods (``wait_for_busy``, ``is_busy``).  Pass ``None`` if
            operating state is not available.
        :param ral: RAL instance used to read the current time and check
            for ROS shutdown.
        """
        self.__inst = class_instance
        self.__ral = ral
        self.__start_time = ral.now()

        if not class_instance:
            self.wait = self._unsupported
            self.is_busy = self._unsupported

    def _unsupported(self, *args, **kwargs):
        raise RuntimeWarning("can't wait, class doesn't support CRTK operating state")

    def wait(self, is_busy = False, timeout = 30.0):
        """Block until the device reaches the requested busy state.

        * ``wait(is_busy=True)`` returns once the device reports
          ``is_busy=True`` for the first time *after* this handle was
          created (i.e. the move has been acknowledged).
        * ``wait(is_busy=False)`` (default) waits for the complete
          busy→idle transition, i.e. it waits for a ``True`` busy state
          followed by a ``False`` busy state.

        Returns ``False`` immediately if ROS has been shut down.

        :param is_busy: If ``True``, wait until the device becomes busy.
            If ``False`` (default), wait until the device becomes idle
            (move complete).
        :type is_busy: bool
        :param timeout: Maximum time in seconds to wait.  Defaults to 30.
        :type timeout: float
        :return: ``True`` if the condition was met within *timeout*,
            ``False`` otherwise (including ROS shutdown).
        :rtype: bool
        """
        if self.__ral.is_shutdown():
            return False

        return self.__inst.wait_for_busy(is_busy = is_busy,
                                       start_time = self.__start_time,
                                       timeout = timeout)

    def is_busy(self, timeout = 30.0):
        """Return whether the device is currently busy with this move.

        Checks the CRTK ``is_busy`` flag relative to the start time
        recorded when this handle was created so that activity from
        previous moves is not counted.

        :param timeout: If the elapsed time since handle creation exceeds
            this value a ``RuntimeWarning`` is raised, indicating that
            the caller is polling too long after the original move
            command.
        :type timeout: float
        :return: ``True`` if the device is busy, ``False`` otherwise.
        :rtype: bool
        :raises RuntimeWarning: If called after *timeout* seconds have
            elapsed since handle creation.
        """
        # if we keep asking past timeout, throw an exception
        if (self.__ral.now() - self.__start_time) > self.__ral.create_duration(timeout):
            raise RuntimeWarning('is_busy() called after timeout')

        return self.__inst.is_busy(start_time = self.__start_time)
