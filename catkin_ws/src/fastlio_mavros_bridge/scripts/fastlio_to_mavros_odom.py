#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def static_tfs(parent, child):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    return t

def cb(msg):
    out = msg
    out.header.frame_id = "odom_ned"
    out.child_frame_id = "base_link_frd"
    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("fastlio_to_mavros_odom")

    in_topic = rospy.get_param("~in_topic", "/Odometry")
    out_topic = rospy.get_param("~out_topic", "/mavros/odometry/out")

    pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    #br = tf2_ros.StaticTransformBroadcaster()
    #br.sendTransform([
    #    static_tfs("odom_ned", "camera_init"),
    #    static_tfs("base_link_frd", "body")
    #])

    rospy.loginfo("Relaying %s -> %s", in_topic, out_topic)
    rospy.spin()
