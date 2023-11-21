#  Author(s):  Anton Deguet
#  Created on: 2022-06-07

# (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import rospy
import PyKDL

class measured_cp(object):
    """Simple class to get measured_cp over ROS
    """

    # initialize the arm
    def __init__(self, ros_namespace, expected_interval = 0.01):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_arm(ros_namespace, expected_interval)


    def __init_arm(self,ros_namespace, expected_interval):
        """Constructor.  This initializes a few data members. It
        requires a ros namespace, this will be used to find the ROS
        topic `measured_cp`."""
        # data members, event based
        self.__ros_namespace = ros_namespace

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ros_namespace, expected_interval)

        # add crtk features that we need
        self.__crtk_utils.add_measured_cp()

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('measured_cp_client', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')
