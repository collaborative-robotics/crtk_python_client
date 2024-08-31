#  Author(s):  Anton Deguet
#  Created on: 2022-06-07

# (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk

class measured_cp(object):
    """Simple class to get measured_cp over ROS
    """

    # initialize the arm
    def __init__(self, ral, expected_interval = 0.01):
        """Constructor.  This initializes a few data members. It
        requires a ros namespace, this will be used to find the ROS
        topic `measured_cp`."""
        # data members, event based
        self.__ral = ral

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        # add crtk features that we need
        self.__crtk_utils.add_measured_cp()
