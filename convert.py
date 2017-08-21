#!/usr/bin/env python

import logging

import rospy
from std_msgs.msg import String
from people_msgs.msg import Person
from people_msgs.msg import People

import rsb
import rst
import rstsandbox

from rst.vision.HeadObject_pb2 import HeadObject
from rsb.converter import ProtocolBufferConverter, registerGlobalConverter

def convert(rosdata):

    person_info = rosdata.data.split(":")

    headobj = HeadObject()

    #head bounding box
    #headobj.region.top_left.x = 1
    #headobj.region.top_left.y = 2
    #headobj.region.width = 3
    #headobj.region.height = 4

    #head pan/tilt
    #headobj.pose.x = 5
    #headobj.pose.y = 6
    #headobj.pose.z = 0

    #headobj.position.y = person_msg.position.y
    #headobj.position.z = person_msg.position.z

    #gender, age
    #personinfo = person_msg.name.split(":")

    #gender = personinfo[0]
    #age = personinfo[1]

    #headobj.gender.decided_class = gender
    #headobj.age.decided_class = age


    try:
        #head position
        headobj.position.x = float(person_info[0])
        headobj.position.y = float(person_info[1])
        headobj.position.z = float(person_info[2])

        #head orientation
        headobj.pose.x = float(math.radians(person_info[3]))
        headobj.pose.y = float(math.radians(person_info[4]))
        headobj.pose.z = 0

    except Exception, e:
        print "Error parsing head positino or gaze directions:"
        print str(e)

    return headobj

def publish(rstdata):
    informer.publishData(rstdata)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received %s', data)
    rstdata = convert(data)
    publish(rstdata)


def ros_listen():
    rospy.init_node('poselistener', anonymous=True)
    rospy.Subscriber('/some/topic', String, callback)
    rospy.spin()

if __name__ == '__main__':

    logging.basicConfig()

    rsb.converter.registerGlobalConverter(rsb.converter.ProtocolBufferConverter(messageClass=HeadObject))
    with rsb.createInformer("/pepper/headpose", dataType=HeadObject) as informer:
        ros_listen()
