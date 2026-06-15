#!/usr/bin/env python3
"""Convert nav_msgs/Odometry to nav_msgs/Path for RViz trajectory display."""
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath:
    def __init__(self):
        self.path = Path()
        self.path.header.frame_id = "world"
        self.pub = rospy.Publisher("/sim/path", Path, queue_size=10, latch=True)
        self.sub = rospy.Subscriber("/sim/odom", Odometry, self.callback)

    def callback(self, msg):
        self.path.header.stamp = msg.header.stamp
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        # Keep last 10000 poses (~50s at 200Hz)
        if len(self.path.poses) > 10000:
            self.path.poses = self.path.poses[-10000:]
        self.pub.publish(self.path)


if __name__ == "__main__":
    rospy.init_node("odom_to_path")
    OdomToPath()
    rospy.spin()
