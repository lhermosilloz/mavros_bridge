#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import copy

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

    # Build PoseWithCovarianceStamped
    out = PoseWithCovarianceStamped()

    # Frames for MAVROS odom plugin -> Mavlink odom
    out.header.frame_id = out.header.frame_id = rospy.get_param("~frame_id", "odom")
    #out.child_frame_id = "base_link"

    # Monotonic-stable stamp (won't jump with NTP)
    dt = monotonic_sec() - t0_mono

    # Add the time stamp
    out.header.stamp = t0_wall + rospy.Duration.from_sec(dt)

    # Copy the pose and covariance from Odometry
    out.pose.pose = latest.pose.pose
    out.pose.covariance = list(latest.pose.covariance)

    # Force the pose covariance diagonals
    pc = list(out.pose.covariance)
    pc[0] = 0.01
    pc[7] = 0.01
    pc[14] = 0.0025
    pc[21] = 0.03
    pc[28] = 0.03
    pc[35] = 0.07
    out.pose.covariance = pc

    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("fastlio_to_mavros_odom_repub")

    in_topic = rospy.get_param("~in_topic", "/Odometry")
    out_topic = rospy.get_param("~out_topic", "/mavros/vision_pose/pose_cov")
    rate_hz = rospy.get_param("~rate_hz", 30.0)

    pub = rospy.Publisher(out_topic, PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    rospy.Timer(rospy.Duration(1.0 / rate_hz), timer_cb)
    rospy.loginfo("Republishing latest %s -> %s at %.1f Hz", in_topic, out_topic, rate_hz)
    rospy.spin()