import PyKDL
import numpy
import geometry_msgs.msg


def FrameFromTransformMsg(t):
    """
    :param t: input transform
    :type t: :class:`geometry_msgs.msg.Transform`
    :return: New :class:`PyKDL.Frame` object

    Convert a transform represented as a ROS Transform message to a :class:`PyKDL.Frame`.
    There must be a standard package to perform this conversion, if you find it, please remove this code.
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x,
                                                 t.rotation.y,
                                                 t.rotation.z,
                                                 t.rotation.w),
                       PyKDL.Vector(t.translation.x,
                                    t.translation.y,
                                    t.translation.z))

def FrameToTransformMsg(f):
    """
    :param f: input frame
    :type f: :class:`PyKDL.Frame`

    Return a ROS TransformStamped message for the Frame f.
    There must be a standard package to perform this conversion, if you find it, please remove this code.
    """
    t = geometry_msgs.msg.TransformStamped()
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = f.M.GetQuaternion()
    t.translation.x = f.p[0]
    t.translation.y = f.p[1]
    t.translation.z = f.p[2]
    return t


def FrameFromPoseMsg(p):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    There must be a standard package to perform this conversion, if you find it, please remove this code.
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(p.orientation.x,
                                                 p.orientation.y,
                                                 p.orientation.z,
                                                 p.orientation.w),
                       PyKDL.Vector(p.position.x,
                                    p.position.y,
                                    p.position.z))

def FrameToPoseMsg(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a ROS PoseStamped message for the Frame f.
    There must be a standard package to perform this conversion, if you find it, please remove this code.
    """
    p = geometry_msgs.msg.Pose()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = f.M.GetQuaternion()
    p.position.x = f.p[0]
    p.position.y = f.p[1]
    p.position.z = f.p[2]
    return p


def ArrayFromTwistMsg(t):
    return numpy.array([t.linear.x,
                        t.linear.y,
                        t.linear.z,
                        t.angular.x,
                        t.angular.y,
                        t.angular.z])


def ArrayFromWrenchMsg(w):
    return numpy.array([w.force.x,
                        w.force.y,
                        w.force.z,
                        w.torque.x,
                        w.torque.y,
                        w.torque.z])


def StampToSeconds(stamp):
    return stamp.sec * 1.0 + stamp.nanosec / 1.e9
