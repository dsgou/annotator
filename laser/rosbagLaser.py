#!/usr/bin/env python
import roslib
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
import signal
import os
import sys
import time
import threading
import rosbag
import yaml
import numpy as np
import matplotlib.pyplot as plt
import argparse
import textwrap
import math
#import qt_laserscan
import graphicalInterfaceLaser as gL


programmName = os.path.basename(sys.argv[0])
laserDistances = []
theta = []
sx = []
sy = []

def play_bag_file(bag, input_topic):
    global laserDistances, sx, sy, theta

    topicKey = 0
    topic = 0
    flag = False
    info_dict = yaml.load(bag._get_yaml_info())
    topics =  info_dict['topics']

    for key in range(len(topics)):
        if topics[key]['topic'] == input_topic:
            topicKey = key

    topic = topics[topicKey]
    messages =  topic['messages']
    duration = info_dict['duration']
    topic_type = topic['type']
    frequency = topic['frequency']


    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate

    bridge = CvBridge()
    image_buff = []
    time_buff = []
    box_buff = []
    counter = 0
    buff_size = messages
    #file_obj = open(feature_file, 'a')

    #Loop through the rosbag
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        #Get the scan
        laserDistances.append(np.array(msg.ranges))
        theta = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        theta = np.degrees(theta)
        sx.append(np.cos(np.radians(theta)) * laserDistances[-1])
        sy.append(np.sin(np.radians(theta)) * laserDistances[-1])
    laserDistances = []

#main
def runMain(bag, bag_file,topic):
    #args = parse_arguments()
    #bag_file = args.input_file
    #csv_file = args.csv_file
    #output_file = args.output_file
    #input_topic = args.scan_topic
    #append = args.append

    #Create results file
    feature_file = bag_file.split(".")[0].split("/")[-1] + "_RESULT"

    if os.path.exists(feature_file) and not append:
        os.remove(feature_file)

    #Open bag and get framerate
    play_bag_file(bag, topic)
    gL.run(sx, sy, bag, bag_file)

