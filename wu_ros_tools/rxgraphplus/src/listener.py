#!/usr/bin/env python
import roslib; roslib.load_manifest('rxgraphplus')
import rospy
from std_msgs.msg import String
import sys

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def listener():
    rospy.init_node('listener%s'%sys.argv[1])
    rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("awesome", String, callback)
    rospy.Publisher("awesome", String)
    rospy.spin()

if __name__ == '__main__':
    listener()
