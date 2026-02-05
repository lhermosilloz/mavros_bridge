#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import copy
import math
import ctypes
import ctypes.util

# Monotonically increasing
t0_wall = None
t0_mono = None

latest = None

# Monotonic clock for Python 2
_librt = ctypes.CDLL(ctypes.util.find_library('rt') or 'librt.so.1', use_errno=True)

class timespec(ctypes.Structure):
    _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]

CLOCK_MONOTONIC = 1

def monotonic_sec():
    t = timespec()
    if _librt.clock_gettime(CLOCK_MONOTONIC, ctypes.pointer(t)) != 0:
        errno = ctypes.get_errno()
        raise OSError(errno, "clock_gettime(CLOCK_MONOTONIC) failed")
    return t.tv_sec + t.tv_nsec * 1e-9

def apply_roll(qx, qy, qz, qw):
    """
    Apply rotation about X axis
    q_flip = (1, 0, 0, 0)
    Post multiply: q_new = q xor q_flip = (w, z, -y, -x)
    """
    nx = qw
    ny = qz
    nz = -qy
    nw = -qx

    # Normalize (safe)
    n = math.sqrt(nx*nx + ny*ny + nz*nz + nw*nw)
    if n > 1e-12:
        nx /= n
        ny /= n
        nz /= n
        nw /= n
    return nx, ny, nz, nw

def cb(msg):
    global latest
    latest = msg

def timer_cb(_evt):
    global latest, t0_wall, t0_mono
    if latest is None:
        return

    if t0_wall is None:
        t0_wall = rospy.Time.now()
        t0_mono = monotonic_sec()

    # Build with /Odometry message
    out = copy.deepcopy(latest)

    # Frames for MAVROS odom plugin -> Mavlink odom
    out.header.frame_id = "odom"
    out.child_frame_id = "base_link"

    # Monotonic-stable stamp (won't jump with NTP)
    dt = monotonic_sec() - t0_mono

    # Add the time stamp
    out.header.stamp = t0_wall + rospy.Duration.from_sec(dt)

    q = out.pose.pose.orientation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    nx, ny, nz, nw = apply_roll(qx, qy, qz, qw)
    q.x, q.y, q.z, q.w = nx, ny, nz, nw

    if all(v == 0.0 for v in out.twist.covariance):
        out.twist.covariance = [1.0] * 36

    # Force the pose covariance diagonals
    #pc = list(out.pose.covariance)
    #pc[0] = 0.01
    #pc[7] = 0.01
    #pc[14] = 0.0025
    #pc[21] = 0.03
    #pc[28] = 0.03
    #pc[35] = 0.07
    #out.pose.covariance = pc

    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("fastlio_to_mavros_odom_base")

    in_topic = rospy.get_param("~in_topic", "/Odometry")
    out_topic = rospy.get_param("~out_topic", "/mavros/odometry/out")
    rate_hz = rospy.get_param("~rate_hz", 30.0)

    pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    rospy.Timer(rospy.Duration(1.0 / rate_hz), timer_cb)
    rospy.loginfo("Republishing latest %s -> %s at %.1f Hz", in_topic, out_topic, rate_hz)
    rospy.spin()