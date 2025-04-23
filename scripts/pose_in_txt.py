#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    # Estrai tempo (sec.nanosec)
    t = msg.header.stamp
    timestamp = f"{t.secs}.{str(t.nsecs).zfill(9)}"

    # Estrai posizione
    p = msg.pose.pose.position
    position = f"{p.x:.6f} {p.y:.6f} {p.z:.6f}"

    # Estrai orientazione (quaternion)
    q = msg.pose.pose.orientation
    orientation = f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}"

    # Riga da salvare
    line = f"{timestamp} {position} {orientation}\n"

    # Scrivi nel file
    with open("logs/pose3_log.txt", "a") as f:
        f.write(line)

    rospy.loginfo(f"Saved: {line.strip()}")

def listener():
    rospy.init_node('poseimu_logger', anonymous=True)
    rospy.Subscriber("/ov_msckf/poseimu", PoseWithCovarianceStamped, callback)
    rospy.loginfo("Listening to /ov_msckf/poseimu...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
