#  Author(s):  Anton Deguet
#  Created on: 2020-11-23

# (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import rospy


class wait_move_handle:
    def __init__(self, class_instance):
        self.__class_instance = class_instance
        self.__start_time = rospy.Time.now()

    def wait(self, is_busy = False, timeout = 30.0):
        if rospy.is_shutdown():
            return False
        if self.__class_instance:
            return self.__class_instance.wait_for_busy(is_busy = is_busy,
                                                       start_time = self.__start_time,
                                                       timeout = timeout)
        else:
            raise RuntimeWarning('can\'t wait, the class doesn\'t support CRTK operating state')
