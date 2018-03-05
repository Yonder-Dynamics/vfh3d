#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from pyquaternion import Quaternion

def init():
    rospy.init_node("vehicle_sim", anonymous=True)
    pub = rospy.Publisher("/vehiclePose", PoseStamped, queue_size=10)
    def callback(msg):
        print(msg)
        q = Quaternion(msg.pose.orientation.w,msg.pose.orientation.x,
                       msg.pose.orientation.y,msg.pose.orientation.z)
        diff = q.rotate(np.array([1, 0, 0]))
        nextPose = PoseStamped()
        nextPose.pose.position.x = msg.pose.position.x + diff[0]
        nextPose.pose.position.y = msg.pose.position.y + diff[1]
        nextPose.pose.position.z = msg.pose.position.z + diff[2]
        nextPose.pose.orientation = msg.pose.orientation
        nextPose.header.frame_id = "map"
        rospy.loginfo(nextPose)
        pub.publish(nextPose)
    rospy.Subscriber("next_direction", PoseStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    init()
