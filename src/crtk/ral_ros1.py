#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023-2024 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rospy
import sys

class ral:
    """RAL: ROS abstraction layer

    Provides many common parts of rospy/rclpy via a common API, to allow CRTK code to
    be written at a higher level, and to work for both ROS 1 and ROS 2.
    """

    @staticmethod
    def ros_version():
        """Return the ROS version this RAL implementation targets.

        :return: Always returns ``1`` for the ROS 1 implementation.
        :rtype: int
        """
        return 1

    @staticmethod
    def parse_argv(argv):
        """Strip ROS-specific arguments from an argument list.

        Uses ``rospy.myargv`` to filter out any ROS remapping or
        parameter arguments so the remaining list can be passed to
        application-level argument parsers.

        :param argv: Argument list (e.g. ``sys.argv``).
        :return: Argument list with ROS arguments removed.
        :rtype: list[str]
        """
        # strip ros arguments
        return rospy.myargv(argv)

    def __init__(self, node_name, namespace = '', node = None):
        """Initialize the ROS 1 abstraction layer.

        If *node* is ``None`` a new ROS node is initialised via
        ``rospy.init_node``.  Passing an existing *node* object allows
        sharing a node across multiple ``ral`` instances.

        :param node_name: Name used when initialising the ROS node.
        :type node_name: str
        :param namespace: Optional topic namespace prefix.  Leading and
            trailing slashes are stripped automatically.
        :type namespace: str
        :param node: Existing ROS node to reuse.  When provided the
            ``rospy.init_node`` call is skipped.
        """
        self._node_name = node_name
        self._namespace = namespace.strip('/')

        self._children = {}
        self._publishers = []
        self._subscribers = []
        self._services = []

        if node is not None:
            self._node = node
        else:
            # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
            rospy.init_node(self.node_name(), anonymous = True)

    def __del__(self):
        for pub in self._publishers:
            pub.unregister()

        for sub in self._subscribers:
            sub.unregister()

    def node_name(self):
        """Return the node name supplied at construction time.

        :rtype: str
        """
        return self._node_name

    def namespace(self):
        """Return the current topic namespace (without leading/trailing slashes).

        :rtype: str
        """
        return self._namespace

    def create_child(self, child_namespace):
        """Create a child ``ral`` object scoped to a sub-namespace.

        The child shares the same ROS node but all its topics are
        automatically prefixed with ``<parent_namespace>/<child_namespace>``.

        :param child_namespace: The sub-namespace name.
        :type child_namespace: str
        :return: New child ``ral`` instance.
        :raises RuntimeError: If a child with *child_namespace* already exists.
        """
        if child_namespace in self._children:
            err_msg = 'ral object already has child "{}"!'.format(child_namespace)
            raise RuntimeError(err_msg)

        full_child_namespace = self._add_namespace_to(child_namespace)
        child = ral(self.node_name(), full_child_namespace, self)
        self._children[child_namespace] = child
        return child

    def now(self):
        """Return the current ROS time.

        :return: Current ROS time.
        :rtype: rospy.Time
        """
        return rospy.Time.now()

    def get_timestamp(self, t):
        """Extract a ``rospy.Time`` stamp from a message or header.

        Accepts a stamped message, a ``Header``, or a plain ``Time``
        object and always returns a ``rospy.Time``.

        :param t: Object to extract the stamp from.
        :return: The extracted time stamp.
        :rtype: rospy.Time
        """
        if hasattr(t, 'header'):
            t = t.header
        if hasattr(t, 'stamp'):
            t = t.stamp

        return t

    def to_sec(self, t):
        """Convert a ROS time or stamped message to seconds (float).

        :param t: A ``rospy.Time``, stamped message, or ``Header``.
        :return: Time in seconds.
        :rtype: float
        """
        t = self.get_timestamp(t)
        return t.to_sec()

    def create_duration(self, d):
        """Create a ROS ``Duration`` from a number of seconds.

        :param d: Duration in seconds.
        :type d: float
        :return: Corresponding ROS duration.
        :rtype: rospy.Duration
        """
        return rospy.Duration(d)

    def create_rate(self, rate_hz):
        """Create a ROS ``Rate`` object for loop-rate control.

        :param rate_hz: Desired loop rate in Hz.
        :type rate_hz: float
        :return: ROS rate object whose ``.sleep()`` keeps the desired frequency.
        :rtype: rospy.Rate
        """
        return rospy.Rate(rate_hz)

    def create_time(self):
        """Create an empty (zero) ROS ``Time`` object.

        :return: Zero-initialised ROS time.
        :rtype: rospy.Time
        """
        return rospy.Time()

    def set_timestamp(self, msg):
        """Stamp a ROS message with the current time.

        Sets ``msg.header.stamp`` to ``now()`` if the message has a
        ``header`` attribute; otherwise the message is returned unchanged.

        :param msg: ROS message to stamp.
        :return: The same message with the header stamp updated.
        """
        if hasattr(msg, 'header'):
            msg.header.stamp = self.now()
        return msg

    def spin(self):
        """No-op in ROS 1.

        Spinning is handled implicitly by rospy; this method exists for
        API compatibility with the ROS 2 ``ral`` implementation.
        """
        pass # Not applicable in ROS 1

    def shutdown(self):
        """Signal ROS shutdown by raising a ``RuntimeError``.

        Mimics the behaviour of ``rclpy.shutdown()`` so that ROS 1 and
        ROS 2 code can be written uniformly.

        :raises RuntimeError: Always, to signal the node should exit.
        """
        # mimic the rclpy behavior
        raise RuntimeError('crtk.ral.shutdown() has been called')

    def spin_and_execute(self, function, *arguments):
        """Execute *function* with *arguments* inside the ROS context.

        In ROS 1 this simply calls the function directly (spinning is
        handled by rospy callbacks).  The signature mirrors the ROS 2
        implementation which needs to start an executor thread first.

        :param function: Callable to invoke.
        :param arguments: Positional arguments forwarded to *function*.
        """
        function(*arguments)

    def on_shutdown(self, callback):
        """Register a callback to be invoked when ROS shuts down.

        :param callback: Zero-argument callable invoked at shutdown.
        """
        rospy.on_shutdown(callback)

    def is_shutdown(self):
        """Return ``True`` if the ROS node has been shut down.

        :rtype: bool
        """
        return rospy.is_shutdown()

    def _add_namespace_to(self, name):
        name = name.strip('/')
        qualified_name = '{}/{}'.format(self.namespace(), name)
        return qualified_name

    def publisher(self, topic, msg_type, queue_size = 10, latch = False, *args, **kwargs):
        """Create a ROS 1 publisher and register it with this RAL instance.

        The topic name is automatically prefixed with the RAL namespace.

        :param topic: Topic name (relative to the namespace).
        :type topic: str
        :param msg_type: ROS message type class.
        :param queue_size: Publisher queue depth.  Defaults to 10.
        :type queue_size: int
        :param latch: If ``True`` the last message is re-sent to new
            subscribers.  Defaults to ``False``.
        :type latch: bool
        :return: The created ``rospy.Publisher``.
        """
        pub = rospy.Publisher(self._add_namespace_to(topic), msg_type,
                              latch = latch, queue_size = queue_size, *args, **kwargs)
        self._publishers.append(pub)
        return pub

    # latch is added to be compatible with ROS2 crtk.ral
    def subscriber(self, topic, msg_type, callback, queue_size = 10, latch = False, *args, **kwargs):
        """Create a ROS 1 subscriber and register it with this RAL instance.

        The topic name is automatically prefixed with the RAL namespace.
        The *latch* parameter is accepted for API compatibility with the
        ROS 2 implementation but has no effect in ROS 1.

        :param topic: Topic name (relative to the namespace).
        :type topic: str
        :param msg_type: ROS message type class.
        :param callback: Message-received callback.
        :param queue_size: Subscriber queue depth.  Defaults to 10.
        :type queue_size: int
        :param latch: Accepted for API compatibility; ignored in ROS 1.
        :type latch: bool
        :return: The created ``rospy.Subscriber``.
        """
        sub = rospy.Subscriber(self._add_namespace_to(topic), msg_type, callback = callback,
                               queue_size = queue_size, *args, **kwargs)
        self._subscribers.append(sub)
        return sub

    def service_client(self, name, srv_type):
        """Create a ROS 1 service client and register it with this RAL instance.

        :param name: Service name (relative to the namespace).
        :type name: str
        :param srv_type: ROS service type class.
        :return: The created ``rospy.ServiceProxy``.
        """
        client = rospy.ServiceProxy(self._add_namespace_to(name), srv_type)
        self._services.append(client)
        return client

    def get_topic_name(self, pubsub):
        """Return the fully-qualified topic name for a publisher or subscriber.

        :param pubsub: A ``rospy.Publisher`` or ``rospy.Subscriber`` instance.
        :return: The topic name string.
        :rtype: str
        """
        return pubsub.name

    def _check_connections(self, start_time, timeout_duration, check_children):
        check_rate = self.create_rate(100)
        connected = lambda pubsub: pubsub.get_num_connections() > 0

        # wait at most timeout_seconds for all connections to establish
        while (self.now() - start_time) < timeout_duration:
            pubsubs = self._publishers + self._subscribers
            unconnected = [ps for ps in pubsubs if not connected(ps)]
            if len(unconnected) == 0:
                break

            check_rate.sleep()

        if check_children:
            # recursively check connections in all child ral objects,
            # using same start time so the timeout period doesn't restart
            for _, child in self._children.items():
                child._check_connections(start_time, timeout_duration, check_children)

        # last check of connection status, raise error if any remain unconnected
        unconnected_pubs = [p for p in self._publishers if not connected(p)]
        unconnected_subs = [s for s in self._subscribers if not connected(s)]
        if len(unconnected_pubs) == 0 and len(unconnected_subs) == 0:
            return

        connected_pub_count = len(self._publishers) - len(unconnected_pubs)
        connected_sub_count = len(self._subscribers) - len(unconnected_subs)
        unconnected_pub_names = ', '.join([self.get_topic_name(p) for p in unconnected_pubs])
        unconnected_sub_names = ', '.join([self.get_topic_name(s) for s in unconnected_subs])

        err_msg = \
        (
            'Timed out waiting for publisher/subscriber connections to establish\n'
            '    Publishers:  {} connected,'
            ' {} not connected\n'
            '                 not connected: [{}]\n\n'
            '    Subscribers: {} connected,'
            ' {} not connected\n'
            '                 not connected: [{}]\n\n'
        )
        err_msg = err_msg.format(connected_pub_count, len(unconnected_pubs), unconnected_pub_names,
                                 connected_sub_count, len(unconnected_subs), unconnected_sub_names)
        raise TimeoutError(err_msg.format(connected_pub_count))

    def check_connections(self, timeout_seconds = 5.0, check_children = True):
        """Check that all publishers are connected to at least one subscriber,
        and that all subscribers are connected to at least on publisher.

        If timeout_seconds is zero, no checks will be done.
        If check_children is True, all children will be checked as well.
        """

        if timeout_seconds == 0.0:
            return

        start_time = self.now()
        timeout_duration = self.create_duration(timeout_seconds)

        self._check_connections(start_time, timeout_duration, check_children)
