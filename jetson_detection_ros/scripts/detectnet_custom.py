#!/usr/bin/env python
#
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
from cv_bridge import CvBridge
import cv2
import numpy as np
import jetson.inference
import jetson.utils
import argparse
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

class ImageDetector:

	def __init__(self):
		rospy.init_node('image_detection',anonymous=True)
		
		#Get rosparams from config file
		camera=rospy.get_param('~camera_topic')
		net=rospy.get_param('~network')
		if net=="ssd-mobilenet-v2" or net=="ssd-inception-v2" or net=="ssd-mobilenet-v1":
			self.label="coco"
			self.coco=rospy.get_param('~class_names')
		else:
			self.label="notcoco"
			
		thresh=rospy.get_param('~threshold')
		self.overlay=rospy.get_param('~overlay')

		self.image_pub = rospy.Publisher("/detection/image", Image, queue_size = 1)
		self.pub_message_output=rospy.Publisher("/detection/bounding_box",String,queue_size=1)

		self.Bridge=CvBridge()
		self.net = jetson.inference.detectNet(net, thresh)
		self.image_sub=rospy.Subscriber(camera, Image,self.callback, queue_size=1)


	def callback(self,msg):
		topic_img=self.Bridge.imgmsg_to_cv2(msg,"bgr8")
		img_rgba = cv2.cvtColor(topic_img, cv2.COLOR_BGR2RGBA).astype(np.float32)
		width  = img_rgba.shape[1]
		height = img_rgba.shape[0]
		cuda_img = jetson.utils.cudaFromNumpy(img_rgba)
		detections = self.net.Detect(cuda_img, width, height, self.overlay)
		metadata=[]
		for detection in detections:
			score=detection.Confidence
			if self.label=="coco":
				className=self.coco[detection.ClassID]
			else:
				className=detection.ClassID
			top=detection.Top
			left=detection.Left
			bottom=detection.Bottom
			right=detection.Right
			bounding_box=[top,left,bottom,right]
			metadata.append([str(className),str(score),str(bounding_box)])      
	
		self.pub_message_output.publish(str(metadata))
		np_img=jetson.utils.cudaToNumpy(cuda_img,width,height,4)
		img = np.uint8(np_img)
		img=cv2.cvtColor(img,cv2.COLOR_RGBA2BGR)
		img=self.Bridge.cv2_to_imgmsg(img,'bgr8')
		self.image_pub.publish(img)      

def main():
	image_detection=ImageDetector()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down')

if __name__=='__main__':
	main()

