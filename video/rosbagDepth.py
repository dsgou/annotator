#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import yaml
import cv2
import os
import rosbag
import time
from termcolor import colored
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rosbagVideo

def printProgress (iteration, total, prefix = '', suffix = '', decimals = 1, barLength = 100):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        barLength   - Optional  : character length of bar (Int)
    """
    formatStr       = "{0:." + str(decimals) + "f}"
    percents        = formatStr.format(100 * (iteration / float(total)))
    filledLength    = int(round(barLength * iteration / float(total)))
    bar             = '█' * filledLength + '-' * (barLength - filledLength)
    sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents, '%', suffix)),
    sys.stdout.flush()
    if iteration == total:
        sys.stdout.write('\n')
        sys.stdout.flush()

"""
Buffers depth and time data from rosbag

Input: 
        -bag        : rosbag
        -input_topic: the image topic of the rosbag
        -compressed : if the topic is compressed

Output:
        -image_buff : list of image frames
        -time_buff  : list of time frames corresponding to each image
"""
def buffer_depth_data(bag, input_topic, compressed):
    image_buff = []
    time_buff  = []
    start_time = None
    bridge     = CvBridge()
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_GRAYSCALE)

        # normalize depth image 0 to 255
        depthImg = np.array(cv_image, dtype=np.float32)
        cv2.normalize(depthImg, depthImg, 0, 255, cv2.NORM_MINMAX)
        image_buff.append(depthImg)
        time_buff.append(t.to_sec() - start_time.to_sec())
    return image_buff, time_buff

def write_depth_video(bagFileName, depthFileName, input_topic):
    
    if not os.path.isfile(depthFileName):
        print colored('Get depth data from ROS', 'green')
        (message_count, compressed, framerate) = rosbagVideo.buffer_video_metadata(bagFileName, input_topic)
        (depth_buffer, time_buff) = buffer_depth_data(bagFileName, input_topic, compressed)
        print colored('Writing depth video at: ', 'yellow'), depthFileName
        
        #Check opencv version
        major = cv2.__version__.split(".")[0]
        if major == '3':
            fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
        else:
            fourcc = cv2.cv.CV_FOURCC('X', 'V' ,'I', 'D')

        height, width = depth_buffer[0].shape

        # 0 for grayscale image
        # non zero values for color frames
        video_writer = cv2.VideoWriter(depthFileName, fourcc, framerate, (width,height), 0)

        if not video_writer.isOpened():
            self.errorMessages(2)
        else:
            for frame in depth_buffer:
                depthFrame = frame.astype('uint8')
                video_writer.write(depthFrame)
            video_writer.release()
        print colored('Depth video written sucessfully', 'yellow')
    else:
        print colored('Loading depth Video', 'yellow')

    return depthFileName


