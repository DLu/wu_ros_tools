#!/usr/bin/python

import roslib; roslib.load_manifest('rosbaglive')
import rospy
import rosbag
import sys

# F#@K IT WE'LL DO IT LIVE

if __name__=='__main__':
    rospy.init_node('rosbaglive')
    bagfn = None
    for arg in sys.argv[1:]:
        if ".bag" in arg:
            bagfn = arg
            break
    if bagfn is None:
        rospy.logerr("No Bag specified!")
        exit(1)

    bag = rosbag.Bag(bagfn)
    pubs = {}
    rospy.loginfo('Start read')
    last = None
    data = []

    for topic, msg, t in bag.read_messages():
        if topic not in pubs:
            pub = rospy.Publisher(topic, type(msg), latch=('map' in topic))
            pubs[topic] = pub

        if t!=last:
            data.append( (t, []) )
            last = t
        data[-1][1].append( (topic, msg) )
    rospy.loginfo('Done read')
    start = rospy.Time.now()
    sim_start = None
    for t, msgs in data:
        now = rospy.Time.now()      
        if sim_start is None:
            sim_start = t
        else:
            real_time = now - start
            sim_time = t - sim_start
            if sim_time > real_time:
                rospy.sleep( sim_time - real_time)

        for (topic, msg) in msgs:
            if 'header' in dir(msg):
                msg.header.stamp = now
            elif 'transforms' in dir(msg):
                for tf in msg.transforms:
                    tf.header.stamp = now
            pub = pubs[topic]
            pub.publish(msg)
        if rospy.is_shutdown():
            break
    bag.close()

