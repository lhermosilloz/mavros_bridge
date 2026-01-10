#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import copy
import math
import numpy as np

latest = None
pub = None

def cb(msg):
    global latest
    latest = msg

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n

def quat_mul(q1, q2):
    # ROS uses (x,y,z,w)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=float)

def quat_to_rot(q):
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
    ], dtype=float)

def euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)

    # q = qz ⊗ qy ⊗ qx (yaw-pitch-roll)
    qx = np.array([sr, 0.0, 0.0, cr], dtype=float)
    qy = np.array([0.0, sp, 0.0, cp], dtype=float)
    qz = np.array([0.0, 0.0, sy, cy], dtype=float)
    q = quat_mul(quat_mul(qz, qy), qx)
    return quat_normalize(q)

def rotate_vec(R, v):
    return np.dot(R, np.asarray(v, dtype=float).reshape(3,)).reshape(3,)

def make_timer_cb(Q_BODY_BASE, R_BASE_BODY, out_frame_id, out_child_frame_id):
    def timer_cb(_evt):
        global latest, pub
        if latest is None:
            return

        out = copy.deepcopy(latest)

        # Rotate orientation: q_out = q_in ⊗ q_body_base
        q_in = np.array([
            out.pose.pose.orientation.x,
            out.pose.pose.orientation.y,
            out.pose.pose.orientation.z,
            out.pose.pose.orientation.w
        ], dtype=float)
        q_in = quat_normalize(q_in)
        q_out = quat_mul(q_in, Q_BODY_BASE)
        q_out = quat_normalize(q_out)

        out.pose.pose.orientation.x = float(q_out[0])
        out.pose.pose.orientation.y = float(q_out[1])
        out.pose.pose.orientation.z = float(q_out[2])
        out.pose.pose.orientation.w = float(q_out[3])

        # Rotate twist vectors from body -> base_link
        v_body = np.array([out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z], dtype=float)
        w_body = np.array([out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z], dtype=float)

        v_base = rotate_vec(R_BASE_BODY, v_body)
        w_base = rotate_vec(R_BASE_BODY, w_body)

        out.twist.twist.linear.x  = float(v_base[0])
        out.twist.twist.linear.y  = float(v_base[1])
        out.twist.twist.linear.z  = float(v_base[2])
        out.twist.twist.angular.x = float(w_base[0])
        out.twist.twist.angular.y = float(w_base[1])
        out.twist.twist.angular.z = float(w_base[2])

        out.header.frame_id = out_frame_id
        out.child_frame_id  = out_child_frame_id

        # Covariance safety (python2-safe)
        all_zero = True
        for v in out.twist.covariance:
            if v != 0.0:
                all_zero = False
                break
        if all_zero:
            out.twist.covariance = [1.0] * 36

        pub.publish(out)
    return timer_cb

if __name__ == "__main__":
    rospy.init_node("fastlio_to_mavros_odom_repub")

    in_topic  = rospy.get_param("~in_topic", "/Odometry")
    out_topic = rospy.get_param("~out_topic", "/mavros/odometry/out")
    rate_hz   = rospy.get_param("~rate_hz", 30.0)

    out_frame_id       = rospy.get_param("~out_frame_id", "odom")
    out_child_frame_id = rospy.get_param("~out_child_frame_id", "base_link")

    roll_deg  = rospy.get_param("~corr_roll_deg", 180.0)  # upside-down fix
    pitch_deg = rospy.get_param("~corr_pitch_deg", 0.0)
    yaw_deg   = rospy.get_param("~corr_yaw_deg", 0.0)

    Q_BODY_BASE = euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    R_BODY_BASE = quat_to_rot(Q_BODY_BASE)
    R_BASE_BODY = R_BODY_BASE.T

    pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    rospy.Timer(rospy.Duration(1.0 / rate_hz),
                make_timer_cb(Q_BODY_BASE, R_BASE_BODY, out_frame_id, out_child_frame_id))

    rospy.loginfo("Republishing %s -> %s @ %.1f Hz | out: %s/%s | corr rpy(deg)=%.1f %.1f %.1f",
                  in_topic, out_topic, rate_hz, out_frame_id, out_child_frame_id, roll_deg, pitch_deg, yaw_deg)
    rospy.spin()
