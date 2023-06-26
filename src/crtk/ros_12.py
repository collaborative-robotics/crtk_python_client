#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rospy


class ros_12:
    """Class used to wrap rospy so we can write code independent of ROS version."""

    @staticmethod
    def ros_version():
        return 1

    @staticmethod
    def parse_argv(argv):
        # strip ros arguments
        return rospy.myargv(argv)

    def __init__(self, node_name, namespace = '', node = None):
        self.__node_name = node_name
        self.__namespace = namespace.strip("/")
        self.__rate_in_Hz = 0.0

        self.__children = {}
        self.__publishers = []
        self.__subscribers = []
        self.__services = []

        if node is not None:
            self.__node = node
        else:
            # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
            rospy.init_node(self.node_name(), anonymous = True)

    def __del__(self):
        for pub in self.__publishers:
            pub.unregister()
        
        for sub in self.__subscribers:
            sub.unregister()

    def create_child(self, child_namespace):
        if child_namespace in self.__children:
            raise RuntimeError('ros_12 already has child "{child_namespace}"!')

        child_namespace = child_namespace.strip('/')
        child_full_namespace = f'{self.namespace()}/{child_namespace}'
        child = ros_12(self.node_name(), child_full_namespace, self)
        self.__children[child_namespace] = child
        return child
    
    def now(self):
        return rospy.Time.now()
    
    def timestamp(self, t):
        if hasattr(t, 'header'):
            t = t.header
        if hasattr(t, 'stamp'):
            t = t.stamp

        return t
    
    def to_secs(self, t):
        return t.to_sec()
    
    def timestamp_secs(self, t):
        stamp = self.timestamp(t)
        return self.to_secs(stamp)
    
    def duration(self, d):
        return rospy.Duration(d)

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
    
    def add_namespace(self, name):
        name = name.strip('/')
        return f'{self.namespace()}/{name}'
    
    def spin(self):
        pass # Not applicable in ROS 1

    def shutdown(self):
        pass # Not applicable in ROS 1
    
    def spin_and_execute(self, function, *arguments):
        function(*arguments)

    def on_shutdown(self, callback):
        rospy.on_shutdown(callback)

    def is_shutdown(self):
        return rospy.is_shutdown()

    def publisher(self, topic, msg_type, queue_size = 10, latch = False, *args, **kwargs):
        pub = rospy.Publisher(self.add_namespace(topic), msg_type,
                              latch = latch, queue_size = queue_size, *args, **kwargs)
        self.__publishers.append(pub)
        return pub

    def subscriber(self, topic, msg_type, callback, queue_size = 10, *args, **kwargs):
        sub = rospy.Subscriber(self.add_namespace(topic), msg_type, callback=callback,
                               queue_size = queue_size, *args, **kwargs)
        self.__subscribers.append(sub)
        return sub
    
    def service_client(self, name, srv_type):
        client = rospy.ServiceProxy(self.add_namespace(name), srv_type)
        self.__services.append(client)
        return client
    
    def get_topic(self, pubsub):
        return pubsub.name

    def check_connections(self, timeout_seconds):
        """Check that all publishers are connected to at least one subscriber,
        and that all subscribers are connected to at least on publisher.
        
        If timeout_seconds is zero, no checks will be done.
        """
        if timeout_seconds <= 0.0:
            return

        start_time = self.now()
        timeout_duration = self.duration(timeout_seconds)
        rate = rospy.Rate(100)

        connected = lambda pubsub: pubsub.get_num_connections() > 0

        # wait at most timeout_seconds for all connections to establish
        while (self.now() - start_time) < timeout_duration:
            pubsubs = self.__publishers + self.__subscribers
            unconnected = [ps for ps in pubsubs if not connected(ps)]
            if len(unconnected) == 0:
                break

            rate.sleep()

        # last check of connection status, raise error if any remain unconnected
        unconnected_pubs = [p for p in self.__publishers if not connected(p)]
        unconnected_subs = [s for s in self.__subscribers if not connected(s)]
        if len(unconnected_pubs) == 0 and len(unconnected_subs) == 0:
            return
        
        connected_pub_count = len(self.__publishers) - len(unconnected_pubs)
        connected_sub_count = len(self.__subscribers) - len(unconnected_subs)
        unconnected_pub_names = ', '.join([p.name for p in unconnected_pubs])
        unconnected_sub_names = ', '.join([s.name for s in unconnected_subs])

        err_msg = \
        (
            f"Timed out waiting for publisher/subscriber connections to establish\n"
            f"    Publishers:  {connected_pub_count} connected,"
            f" {len(unconnected_pubs)} not connected\n"
            f"                 not connected: [{unconnected_pub_names}]\n\n"
            f"    Subscribers: {connected_sub_count} connected,"
            f" {len(unconnected_subs)} not connected\n"
            f"                 not connected: [{unconnected_sub_names}]\n\n"
        )
        raise TimeoutError(err_msg)
