#!/usr/bin/env python
import rospy
from vizzy_fingers.msg import *
from sensor_msgs.msg import *
import message_filters

file1 = open("dados.txt","w")

def callback(image_sub, info_sub):
    print 'Callback'
    test1 = image_sub.joint_angles
    test2 = info_sub.position[27]
    seconds = str(info_sub.header.stamp.secs)
    nanoseconds = str(info_sub.header.stamp.nsecs)
    file1.write(" s " + seconds + " " + " ns " + nanoseconds + " p " + str(test1) + " , " + str(test2) + "\n")

#    print str(test1)
#    print str(test2)
    
#def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

if __name__ == '__main__':
    #listener()
    rospy.init_node('listener', anonymous=True)
    image_sub = message_filters.Subscriber('joint_angles_fingers_topic', vizzy_fingers.msg.JointAngles)
    info_sub = message_filters.Subscriber('vizzy/joint_states', sensor_msgs.msg.JointState)
    #ts = message_filters.TimeSynchronizer([image_sub, info_sub], 100)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print 'Done'
    #rospy.Subscriber("joint_angles_fingers_topic", vizzy_fingers.msg.JointAngles, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

