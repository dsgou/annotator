#!/usr/bin/env python

import os
import csv
import cv2
import ast
import subprocess
import numpy as np

from termcolor import colored
from cv_bridge import CvBridge, CvBridgeError

import rosbagVideo

"""
Buffers image and time data from rosbag

Input: 
        -bag        : rosbag
        -input_topic: the image topic of the rosbag
        -compressed : if the topic is compressed

Output:
        -image_buff : list of image frames
"""
def buffer_rgb_data(bag, input_topic, compressed):
    image_buff = []
    start_time = None
    bridge     = CvBridge()
    #Buffer the images, timestamps from the rosbag
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
	
        image_buff.append(cv_image)

    return image_buff
  
"""
Buffers csv data for the video module, a.k.a. bounded boxes and 
their metrics

Input: 
        -csv_file : the path of the csv

Output:
        -box_buff       : list of image frames
        -box_buff_action: list of time frames corresponding to each image
"""
def buffer_video_csv(csv_file):
    features 	    = []
    headlines 	    = []
    box_buff   	    = []
    box_buff_action = []
    if csv_file is not None and os.path.exists(csv_file):
        with open(csv_file, 'r') as file_obj:
	    try:
		csv_reader = csv.reader(file_obj, delimiter = '\t')
		headlines = next(csv_reader)
		headlines = filter(None, headlines)
		for row in csv_reader:
		    timestamp = float(row[0])
		    if len(row) > 2:
			(rec_id, x, y, width, height) = map(int, row[1:6])
			box_buff.append((timestamp, rec_id, x, y, width, height))
			if 'Class' in headlines:
			    features.append(map(float, row[6:-1]))
			    string = ast.literal_eval(row[-1])
			    box_buff_action.append(string)
			else:
			    features.append(map(float, row[6::]))
			    box_buff_action.append(["Clear"])
			    
		    else:
			box_buff_action.append(["Clear"])
			box_buff.append((timestamp, -1, 0, 0, 0, 0))
			features.append([0])
		if 'Class' not in headlines:
		    headlines.append('Class')
	    except:
               print("Error processing video csv")
	       
    return headlines, box_buff, box_buff_action, features

"""
Writes rgb video from buffer to selected path

Input: 
        -rgbFileName : path to write the video
        -image_buffer: buffer containing video frames

Output: --
"""
def write_rgb_video(rgbFileName, image_buffer, framerate):
	result = False
	print  colored('Writing rgb video at: ', 'yellow'),rgbFileName 
	#Check opencv version
	major = cv2.__version__.split(".")[0]
	if major == '3':
		fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
	else:
		fourcc = cv2.cv.CV_FOURCC('X', 'V' ,'I', 'D')

	height, width, bytesPerComponent = image_buffer[0].shape
	
	video_writer = cv2.VideoWriter(rgbFileName, fourcc, framerate, (width,height), cv2.IMREAD_COLOR)
	
	if video_writer.isOpened():
		result = True
		i= 0
		for frame in image_buffer:
			#~ cv2.imwrite("/home/dimitris/projectsPython/RoboMAE/robomae/test/" + str(i) + ".png", frame)
			i +=1
			video_writer.write(frame)
		video_writer.release()
		print colored('Video writen successfully', 'yellow')
	#~ cap = cv2.VideoCapture(rgbFileName)
	#~ i= 0
	#~ while(True):
	    
	    #~ if cap is not None:
		#~ ret, frame = cap.read()
		#~ if frame is None:
		    #~ cap.release()
		    #~ break
		#~ cv2.imwrite("/home/dimitris/projectsPython/RoboMAE/robomae/test/" + str(i) + ".png", frame)
		#~ i +=1	    	
	return result

def get_metadata(input_video):
    input_video = input_video.replace(" ", "\ ")
    
    result = subprocess.Popen('ffprobe -i ' + str(input_video) + ' -show_entries format=duration -v quiet -of csv="p=0"', stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
    duration = float(result.communicate()[0])
    result = subprocess.Popen('ffprobe -v error -select_streams v:0 -show_entries stream=avg_frame_rate -of default=noprint_wrappers=1:nokey=1 ' + str(input_video), stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
    frame_info = result.communicate()[0]
    framerate = float(frame_info.split('/')[0])/float(frame_info.split('/')[1])
    result = subprocess.Popen('ffprobe -v error -select_streams v:0 -show_entries stream=nb_frames -of default=noprint_wrappers=1:nokey=1 ' + str(input_video), stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
    frame_count = int(result.communicate()[0])
    return duration, framerate, frame_count
	    
