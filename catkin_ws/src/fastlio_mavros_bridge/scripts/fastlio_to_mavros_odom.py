#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import copy

latest = None

def cb(msg):
    global latest
    latest = msg

def timer_cb(_evt):
    global latest
    if latest is None:
        return
    out = copy.deepcopy(latest)

    # Frames for MAVROS odometry plugin -> MAVLink ODOMETRY
    out.header.frame_id = "odom"
    out.child_frame_id = "base_link"

    # IMPORTANT: Never send "perfect" covariances (0). Make them non-zero.
    # If you don't have twist, at least make twist covariance large (so it doesn't get trusted).
    # nav_msgs/Odometry: pose.covariance and twist.covariance are 6x6 flattened.
    if all(v == 0.0 for v in out.twist.covariance):
        out.twist.covariance = [1.0] * 36

    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("fastlio_to_mavros_odom_repub")

    in_topic = rospy.get_param("~in_topic", "/Odometry")
    out_topic = rospy.get_param("~out_topic", "/mavros/odometry/out")
    rate_hz = rospy.get_param("~rate_hz", 30.0)

    pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    rospy.Timer(rospy.Duration(1.0 / rate_hz), timer_cb)
    rospy.loginfo("Republishing latest %s -> %s at %.1f Hz", in_topic, out_topic, rate_hz)
    rospy.spin()
