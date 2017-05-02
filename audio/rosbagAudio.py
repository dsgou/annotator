#!/usr/bin/env python
import roslib
import cv2

from std_msgs.msg import String
import signal
import os
import sys
import time
import threading
import rosbag
import yaml
import numpy as np
import argparse
import textwrap
import struct
import wave
import subprocess
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms

import runFunction
from audioGlobals import audioGlobals
from termcolor import colored

programmName = os.path.basename(sys.argv[0])

#input parameters
def parse_arguments():
    inputFile = sys.argv[-1]
    #print inputFile 
    return inputFile
    

#isolate audio from bag file
def audio_bag_file(bagFile):
    #save topic and message    
    topicKey = 0
    topic = 0
    flag = False
    bag = bagFile
    info_dict = yaml.load(bag._get_yaml_info())
    topics =  info_dict['topics']

    for key in range(len(topics)):
        if topics[key]['topic'] == '/audio':
            topicKey = key

    topic = topics[topicKey]
    messages =  topic['messages']
    duration = info_dict['duration']
    topic_type = topic['type']
    frequency = topic['frequency']

    audioCheck = topic['topic']
    #Checking if the topic is audio
    if 'audio_common_msgs' in topic_type:
        sound = True
        #print "Audio found"
    else:
        sound = False
        #print "Such a pity..."

    audio = []
    if sound == True:
        for topic, msg, t in bag.read_messages(topics=['/audio']):
            audio += msg.data
    nBytes = len(audio)
    nSamples = nBytes / 2    
    #print "total number of compressed bytes {0:d}".format(nBytes)
    #print "total number of compressed bytes {0:d}".format(nSamples)
    #print "total duration {0:.2f}".format(duration)
    #print "average bit rate {0:.2f}".format(float(nBytes) * 8.0 / float(duration))

    frequency = frequency * 1000
    #bag.close()    
    return audio, frequency

#save mp3 file
def write_mp3_file(audioData, bagFile):
    mp3FileName = bagFile.replace(".bag",".mp3")
    mp3_file = open(mp3FileName, 'w')
    mp3_file.write(''.join(audioData))
    mp3_file.close()
    return mp3FileName

#convert mp3 to wav
def mp3_to_wav(mp3Path, frequency):
    #call arg not file...
    wavFileName = mp3Path.replace(".mp3",".wav")
    #write 1.6 kHz
    subprocess.call(['ffmpeg', '-loglevel', 'quiet', '-i', mp3Path, '-y', '-ar', '16000', '-ac', '1', wavFileName])
    return wavFileName

#convert mp4 to wav
def video_to_wav(video_path, frequency):
    #call arg not file...
    name, extension = os.path.splitext(video_path)
    wavFileName = name + ".wav"
    #write 1.6 kHz
    subprocess.call(['ffmpeg', '-loglevel', 'quiet', '-i', video_path, '-y', '-ar', '16000', '-vn', '-ac', '1', wavFileName])
    return wavFileName

#play wav file
def play_wav(wavFileName):
    #-nodisp   : display not in new window
    #-autoexit : stop automatically
    #-ss       : start time
    #-t        : duration
    subprocess.call(['ffplay', '-nodisp', '-autoexit','-ss','0', '-t', '3.5', wavFileName])


#Plot waveform in GUI
#create waveform of wav file
def createWaveform(wavFileName):
    spf = wave.open(wavFileName,'r')
    #Extract Raw Audio from Wav File
    signal = spf.readframes(-1)
    signal = np.fromstring(signal, 'Int16')

    #If Stereo
    if spf.getnchannels() == 2:
        print 'Just mono files'
        sys.exit(0)

    fig = plt.figure(1)
    plt.title('Signal Wave...')
    plt.plot(signal)
    play_wav(wavFileName)
    
    plt.grid()
    plt.show()



#main
def runMain(bag, fileName):
    video_extensions = [".mp4", ".avi", ".mkv"]
    if bag:
        #read bag file
        audioGlobals.bagFile = fileName
        audioFileName = fileName.replace(".bag",".wav")
        if os.path.isfile(audioFileName):
            print colored('Load WAV File', 'yellow')
            audioGlobals.wavFileName =  audioFileName
            audioGlobals.saveAudio = True
        else:
            print colored('Get audio data from ROS', 'green')
            audioData, frequency = audio_bag_file(bag) 
            # get audio data 
            mp3FileName = write_mp3_file(audioData, audioGlobals.bagFile)
            audioGlobals.wavFileName = mp3_to_wav(mp3FileName, frequency)
            audioGlobals.saveAudio = False
    else:
        name, extension =  os.path.splitext(fileName)
        audioGlobals.saveAudio = False
        if extension == ".wav":
            print colored('Load WAV File', 'yellow')
            audioGlobals.wavFileName = fileName
        elif extension == ".mp3":
            audioGlobals.wavFileName = video_to_wav(fileName, 16000)
        elif extension in video_extensions:
            audioGlobals.wavFileName = video_to_wav(fileName, 16000)
    print audioGlobals.wavFileName
    runFunction.run(audioGlobals.wavFileName, audioGlobals.bagFile)
    


