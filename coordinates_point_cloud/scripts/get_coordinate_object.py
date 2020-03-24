#!/usr/bin/env python2

"""
This code gets a ROS message with bounding boxes coordinates for each object detected 
by the mask RCNN and get the XYZ coordinates of the bounding box center 
in order to generate a TF on it

Written by Thomas CURE and Simon ERNST
"""


import rospy
from sensor_msgs.msg import PointCloud2
import os
import numpy
import ros_numpy as rosnp
from std_msgs.msg import String
from tf_broadcaster.msg import DetectionCoordinates, PointCoordinates

class GetCenterCoordinates(object):
    def __init__(self):
        """
        Create an instance of GetCenterCoordinates Class.
        Setup the ROS node and the parameters used in the code
        """
        rospy.init_node("centercalcul",anonymous=True)
		
	camera_depth_topic=rospy.get_param('~depth_topic')
	bbox=rospy.get_param('~boundingbox_sub')
	output_topic=rospy.get_param('~result_topic')

        self.sub=rospy.Subscriber(camera_depth_topic, PointCloud2, self.handle_sub)
        self.sub_result_msg=rospy.Subscriber(bbox,String,self.handle_result_msg)
        self.cloud_msg=PointCloud2()
        self.rate=rospy.Rate(10)
        self.center_coordinates=None
        self.pub_message=rospy.Publisher(output_topic,DetectionCoordinates,queue_size=1)

    def handle_result_msg(self,req):
        """
        Get the message with the bounding boxes coordinates.
        Extract the data from the message and give the pixel coordinates (height,width).
        Call the get_center_coordinates function
        """
        info=req.data
        res = eval(info)
        if len(res)>0:
            list_label=[]
            list_height=[]
            list_width=[]
            data_array=numpy.zeros((len(res)),dtype={'names':('label','center_height','center_width'),'formats':('U30',numpy.int32,numpy.int32)})
            for object in res:
		print(object)
                list_label.append(str(object[0]))
                coord=eval(object[2])
                center=[int(coord[2])-(int(coord[2])-int(coord[0]))/2,int(coord[-1])-(int(coord[-1])-int(coord[1]))/2]
                list_height.append(center[0])
                list_width.append(center[1])
            data_array['label']=list_label
            data_array['center_height']=list_height
            data_array['center_width']=list_width
            self.get_centers_coordinates(data_array)
        

    def handle_sub(self,req):
        """
        Get the data of the Point Cloud from the kinect
        """
        self.cloud_msg=req


    def get_centers_coordinates(self,data_array):
        """
        Get a data array with the label of each object and the pixel "coordinates" of its center,
        For each center, the program checks in the Point Cloud data to get XYZ coordinates
        related to the kinect system.
        Will publish a custom message with a table containing each point XYZ with its label
        """
        nb_centers=data_array.shape
        self.center_coordinates=numpy.zeros((nb_centers[0]),dtype={'names':('label','x','y','z'),'formats':('U30',numpy.float32,numpy.float32,numpy.float32)})
        array=rosnp.point_cloud2.pointcloud2_to_xyz_array(self.cloud_msg,False)
	print(len(array))
	print(len(array[0]))
        list_label=[]
        list_center_x=[]
        list_center_y=[]
        list_center_z=[]
        for object in data_array:
            list_label.append(object['label'])
            list_center_x.append(array[object['center_height']][object['center_width']][0])
            list_center_y.append(array[object['center_height']][object['center_width']][1])
            list_center_z.append(array[object['center_height']][object['center_width']][2])
        self.center_coordinates["label"]=list_label
        self.center_coordinates["x"]=list_center_x
        self.center_coordinates["y"]=list_center_y
        self.center_coordinates["z"]=list_center_z

        print(self.center_coordinates)

        message=DetectionCoordinates()
        message.header.stamp=rospy.Time.now()
        for object in data_array:
            point=PointCoordinates()
            name=object['label']
            point.name=name.encode('utf-8')
            point.x=array[object['center_height']][object['center_width']][0]
            point.y=array[object['center_height']][object['center_width']][1]
            point.z=array[object['center_height']][object['center_width']][2]
            message.points.append(point)

        self.pub_message.publish(message)


if __name__=="__main__":
    a=GetCenterCoordinates()
    while not rospy.is_shutdown():
        a.rate.sleep()




