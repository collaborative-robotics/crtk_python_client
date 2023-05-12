#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rospy

class ros_12:
    """Class used to wrap rospy features so we can write code independent
    of ROS version.
    """

    def __init__(self, node_name, namespace = ''):
        self.__node_name = node_name
        self.__namespace = namespace
        self.__rate_in_Hz = 0.0
        # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
        rospy.init_node(self.__node_name, anonymous = True)

    @staticmethod
    def ros_version():
        return 1

    @staticmethod
    def parse_argv(argv):
        # strip ros arguments
        return rospy.myargv(argv)

    def set_rate(self, rate_in_Hz):
        self.__rate_in_Hz = rate_in_Hz
        self.__ros_rate = rospy.Rate(rate_in_Hz)

    def sleep(self):
        if self.__rate_in_Hz == 0.0:
            raise RuntimeError('set_rate must be called before sleep')
        self.__ros_rate.sleep()

    def node_name(self):
        return self.__node_name

    def namespace(self):
        return self.__namespace

    def spin_and_execute(self, function):
        function()
