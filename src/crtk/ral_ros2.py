#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023-2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rclpy
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.time
import rclpy.utilities
import threading


class ral:
    """RAL: ROS abstraction layer

    Provides many common parts of rospy/rclpy via a common API, to allow CRTK code to
    be written at a higher level, and to work for both ROS 1 and ROS 2.
    """

    @staticmethod
    def ros_version():
        """Return the ROS version this RAL implementation targets.

        :return: Always returns ``2`` for the ROS 2 implementation.
        :rtype: int
        """
        return 2

    @staticmethod
    def parse_argv(argv):
        """Initialise rclpy and strip ROS-specific arguments from an argument list.

        Calls ``rclpy.init`` then uses ``rclpy.utilities.remove_ros_args``
        to filter out ROS remapping or parameter arguments so the remaining
        list can be passed to application-level argument parsers.

        :param argv: Argument list (e.g. ``sys.argv``).
        :return: Argument list with ROS arguments removed.
        :rtype: list[str]
        """
        # strip ros arguments
        rclpy.init(args = argv)
        return rclpy.utilities.remove_ros_args(argv)

    def __init__(self, node_name, namespace = '', node = None):
        """Initialize the ROS 2 abstraction layer.

        If *node* is ``None`` a new rclpy node is created (initialising
        rclpy if not already done).  Passing an existing *node* object
        allows sharing a node across multiple ``ral`` instances.

        :param node_name: Name used when creating the ROS 2 node.
        :type node_name: str
        :param namespace: Optional topic namespace prefix.  Leading and
            trailing slashes are stripped automatically.
        :type namespace: str
        :param node: Existing rclpy ``Node`` to reuse.  When provided
            node creation is skipped.
        """
        self._node_name = node_name
        self._namespace = namespace.strip('/')

        self._children = {}
        self._publishers = []
        self._subscribers = []
        self._services = []

        self._executor_thread = None

        if node is not None:
            self._node = node
        else:
            # initializes rclpy only if it hasn't been already (e.g. via parse_argv()),
            # otherwise does nothing (since rclpy will raise a RuntimeError)
            try:
                rclpy.init()
            except:
                pass
            # ros init node
            self._node = rclpy.create_node(self.node_name())

    def __del__(self):
        for pub in self._publishers:
            self._node.destroy_publisher(pub)

        for sub in self._subscribers:
            self._node.destroy_subscription(sub)

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

        The child shares the same rclpy ``Node`` but all its topics are
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
        child = ral(self.node_name(), full_child_namespace, self._node)
        self._children[child_namespace] = child
        return child

    def now(self):
        """Return the current ROS 2 time from the node's clock.

        :return: Current ROS 2 time.
        :rtype: rclpy.time.Time
        """
        return self._node.get_clock().now()

    def get_timestamp(self, t):
        """Extract an ``rclpy.time.Time`` stamp from a message or header.

        Accepts a stamped message, a ``Header``, or a plain time value
        and always returns an ``rclpy.time.Time`` object.

        :param t: Object to extract the stamp from.
        :return: The extracted time stamp.
        :rtype: rclpy.time.Time
        """
        if hasattr(t, 'header'):
            t = t.header
        if hasattr(t, 'stamp'):
            t = t.stamp

        try:
            return rclpy.time.Time.from_msg(t)
        except:
            return t

    def to_sec(self, t):
        """Convert a ROS 2 time or stamped message to seconds (float).

        :param t: An ``rclpy.time.Time``, stamped message, or ``Header``.
        :return: Time in seconds.
        :rtype: float
        """
        t = self.get_timestamp(t)
        return float(t.nanoseconds)/1e9

    def create_duration(self, d):
        """Create an ``rclpy.time.Duration`` from a number of seconds.

        :param d: Duration in seconds.
        :type d: float
        :return: Corresponding ROS 2 duration.
        :rtype: rclpy.time.Duration
        """
        return rclpy.time.Duration(seconds = d)

    def create_rate(self, rate_hz):
        """Create a ROS 2 ``Rate`` object for loop-rate control.

        :param rate_hz: Desired loop rate in Hz.
        :type rate_hz: float
        :return: Rate object whose ``.sleep()`` keeps the desired frequency.
        """
        return self._node.create_rate(frequency = rate_hz,
                                      clock = self._node.get_clock())

    def create_time(self):
        """Create a zero-initialised ROS 2 ``Time`` object.

        The clock type matches the node's clock so the value can be
        compared against times returned by :meth:`now`.

        :return: Zero-initialised ROS 2 time.
        :rtype: rclpy.time.Time
        """
        return rclpy.time.Time(seconds = 0.0,
                               clock_type = self._node.get_clock().clock_type)

    def set_timestamp(self, msg):
        """Stamp a ROS 2 message with the current time.

        Sets ``msg.header.stamp`` to ``now().to_msg()`` if the message
        has a ``header`` attribute; otherwise the message is returned
        unchanged.

        :param msg: ROS 2 message to stamp.
        :return: The same message with the header stamp updated.
        """
        if hasattr(msg, 'header'):
            msg.header.stamp = self.now().to_msg()
        return msg

    def _try_spin(self):
        try:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass
        
    def spin(self):
        """Start the rclpy executor in a background daemon thread.

        Uses a ``SingleThreadedExecutor`` so that subscriber callbacks
        are dispatched from a dedicated thread.  Calling ``spin()`` a
        second time on the same instance is a no-op.
        """
        if self._executor_thread != None:
            return
        self._executor_thread = threading.Thread(target = self._try_spin,
                                                 daemon = True)
        self._executor_thread.start()

    def shutdown(self):
        """Shut down rclpy and join the executor thread (if running).

        Safe to call even if the executor has not been started.
        """
        if rclpy.ok():
            rclpy.shutdown()

        if self._executor_thread != None:
            self._executor_thread.join()
            self._executor_thread = None

    def spin_and_execute(self, function, *arguments):
        """Start the executor, run *function*, then shut down.

        Starts the background executor thread via :meth:`spin`, invokes
        *function* with *arguments*, and calls :meth:`shutdown` when
        *function* returns or the user presses Ctrl-C.

        :param function: Callable to invoke.
        :param arguments: Positional arguments forwarded to *function*.
        """
        self.spin()
        try:
            function(*arguments)
        except KeyboardInterrupt:
            pass
        self.shutdown()

    def on_shutdown(self, callback):
        """Register a callback to be invoked when rclpy shuts down.

        :param callback: Zero-argument callable invoked at shutdown.
        """
        rclpy.get_default_context().on_shutdown(callback)

    def is_shutdown(self):
        """Return ``True`` if rclpy has been shut down.

        :rtype: bool
        """
        return not rclpy.ok()

    def _add_namespace_to(self, name):
        name = name.strip('/')
        qualified_name = '{}/{}'.format(self.namespace(), name)
        return qualified_name

    def publisher(self, topic, msg_type, queue_size = 10, latch = False, *args, **kwargs):
        """Create a ROS 2 publisher and register it with this RAL instance.

        The topic name is automatically prefixed with the RAL namespace.
        When *latch* is ``True`` the QoS durability is set to
        ``TRANSIENT_LOCAL`` so that the last *queue_size* messages are
        re-sent to late-joining subscribers.

        :param topic: Topic name (relative to the namespace).
        :type topic: str
        :param msg_type: ROS 2 message type class.
        :param queue_size: Publisher queue depth.  Defaults to 10.
        :type queue_size: int
        :param latch: If ``True`` enables transient-local durability.
            Defaults to ``False``.
        :type latch: bool
        :return: The created rclpy publisher.
        """
        history = rclpy.qos.HistoryPolicy.KEEP_LAST
        qos = rclpy.qos.QoSProfile(depth = queue_size, history = history)
        if latch:
            # publisher will retain queue_size messages for future subscribers
            qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        pub = self._node.create_publisher(msg_type, self._add_namespace_to(topic), qos)
        self._publishers.append(pub)
        return pub

    def subscriber(self, topic, msg_type, callback, queue_size=10, latch = False, *args, **kwargs):
        """Create a ROS 2 subscriber and register it with this RAL instance.

        The topic name is automatically prefixed with the RAL namespace.
        When *latch* is ``True`` the QoS durability is set to
        ``TRANSIENT_LOCAL`` so that previously published messages are
        received immediately upon subscription.

        :param topic: Topic name (relative to the namespace).
        :type topic: str
        :param msg_type: ROS 2 message type class.
        :param callback: Message-received callback.
        :param queue_size: Subscriber queue depth.  Defaults to 10.
        :type queue_size: int
        :param latch: If ``True`` enables transient-local durability.
            Defaults to ``False``.
        :type latch: bool
        :return: The created rclpy subscription.
        """
        history = rclpy.qos.HistoryPolicy.KEEP_LAST
        qos = rclpy.qos.QoSProfile(depth = queue_size, history = history)
        if latch:
            # subscriber should get past messages
            qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        sub = self._node.create_subscription(msg_type, self._add_namespace_to(topic),
                                              callback, qos, *args, **kwargs)
        self._subscribers.append(sub)
        return sub

    def service_client(self, name, srv_type):
        """Create a ROS 2 service client and register it with this RAL instance.

        A ``__call__`` attribute pointing to ``client.call`` is added
        when the client is not directly callable, ensuring a uniform
        call interface across ROS versions.

        :param name: Service name (relative to the namespace).
        :type name: str
        :param srv_type: ROS 2 service type class.
        :return: The created rclpy service client.
        """
        client = self._node.create_client(srv_type, self._add_namespace_to(name))
        if not hasattr(client, '__call__'):
            client.__call__ = client.call
        self._services.append(client)
        return client

    def get_topic_name(self, pubsub):
        """Return the fully-qualified topic name for a publisher or subscriber.

        :param pubsub: An rclpy publisher or subscription instance.
        :return: The topic name string.
        :rtype: str
        """
        return pubsub.topic_name

    def _check_connections(self, start_time, timeout_duration, check_children):
        def connected(pubsub):
            if isinstance(pubsub, rclpy.publisher.Publisher):
                # get_subscription_count() available in >= ROS 2 Dashing
                return pubsub.get_subscription_count() > 0
            elif isinstance(pubsub, rclpy.subscription.Subscription):
                try:
                    return pubsub.get_publisher_count() > 0
                except AttributeError:
                    # get_publisher_count() not available until ROS 2 Iron Irwini
                    return True
            else:
                raise TypeError("pubsub must be an rclpy Publisher or Subscription")

        check_rate = self.create_rate(100)

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
        raise TimeoutError(err_msg)

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
