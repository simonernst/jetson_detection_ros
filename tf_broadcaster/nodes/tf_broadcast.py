#!/usr/bin/env python2

"""
This program will get a ROS custom message in which we can find each bounding box center's
XYZ coordinates for a detected object and associate a TF to the center coordinates 

Written by Thomas CURE and Simon ERNST
"""

import roslib
roslib.load_manifest('tf_broadcaster')
import rospy
import tf
import random
from tf_broadcaster.msg import DetectionCoordinates, PointCoordinates
import math

class ObjectTfBroadcaster(object):
    def __init__(self):
        """
        Create an instance of ObjectTfBroadcaster Class.
        Setup the ROS node and the program's parameters
        """
        rospy.init_node('tfbroadcaster')
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)
	variable=rospy.get_param('~result_topic')
	self.tf_frame=rospy.get_param('~tf_frame_topic')
	self.sub_detection_object=rospy.Subscriber(variable,DetectionCoordinates,self.handle_message_objects)

    def handle_message_objects(self,req):
        """
        Get the ROS message with the XYZ coordinates and publish a TF with the XYZ point as origin
        """
        if len(req.points)>0:
	    i=1
            for point in req.points:
                pos_x=point.x
                pos_y=point.y
                pos_z=point.z
                tf_name=str(point.name)+str('   ')+str(i)
                print(tf_name)
                if math.isnan(pos_x)==False and math.isnan(pos_y)==False and math.isnan(pos_z)==False:
                    self.br.sendTransform((pos_x, pos_y, pos_z),
                                (0.0, 0.0, 0.0, 1.0),
                                rospy.Time.now(),
                                tf_name,
                                self.tf_frame)
		i+=1


if __name__ == '__main__':
    
    a=ObjectTfBroadcaster()
    while not rospy.is_shutdown():
        a.rate.sleep()
