#!/usr/bin/env python
# -*- coding: utf-8 -*-

import yaml
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

"""
Gets the rosbag metadata

Input: 
        -bag        : rosbag
        -input_topic: the topic to get metadata from

Output:
        -messages   : total number of inpu_topic msgs
        -compressed : if the image topic is compressed
        -framerate  : image topic framerate
"""

def buffer_video_metadata(bag, input_topic):
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
    framerate = messages/duration
    return messages, compressed, framerate
  
