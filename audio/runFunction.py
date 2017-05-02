#!/usr/bin/env python
from __future__ import unicode_literals
import sys
import os
import os.path
import random
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor

matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QFont, QPainter
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QFile, QIODevice, QObject, QRect
from PyQt5.QtMultimedia import (QMediaContent,
        QMediaMetaData, QMediaPlayer, QMediaPlaylist, QAudioOutput, QAudioFormat)
from PyQt5.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QPushButton,
        QSizePolicy, QVBoxLayout, QWidget, QToolTip, QLabel, QFrame, QGridLayout, QMenu, qApp, QLineEdit)


from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

from numpy import arange, sin, pi

import wave
import numpy as np
import subprocess
import csv
import cv2
from pyAudioAnalysis import audioFeatureExtraction as aF
from pyAudioAnalysis import audioSegmentation as aS
from pyAudioAnalysis import audioBasicIO

from audioGlobals import audioGlobals
import graphicalInterfaceAudio as gui

def run(wavFileName2,bagFile2):
    time = 0
    segmentDuration = 0
    segments = []

    # >> Open WAVfile 
    #----------------------
    #audioGlobals.wavFileName -> global variable 
    audioGlobals.wavFileName = wavFileName2
    audioGlobals.bagFile = bagFile2

    audioGlobals.spf = wave.open(audioGlobals.wavFileName,'r')
    #Extract Raw Audio from Wav File
    audioGlobals.signal = audioGlobals.spf.readframes(-1)
    audioGlobals.signal = np.fromstring(audioGlobals.signal, 'Int16')
    #self.axes.clear()

    #Get wavFile audioGlabals.duration
    frames = audioGlobals.spf.getnframes()
    rate = audioGlobals.spf.getframerate()
    audioGlobals.duration = frames / float(rate)

    # >> Open CSVfile 
    #----------------------
    # check if .csv exists
    csvFileName = audioGlobals.wavFileName.replace(".wav","_audio.csv")
    if os.path.isfile(csvFileName):
        annotationFile = open(csvFileName, 'rb')

        read = csv.reader(annotationFile)
        for row in read:
            row[0] = float(row[0])
            row[1] = float(row[1])
            audioGlobals.annotations.append([row[0], row[1], row[2]])

        # get speakers unic colors for annotation plot and ganttChart
        #print len(audioGlobals.GreenShades)
        for shadeIndex in range(len(audioGlobals.annotations)):
            if audioGlobals.annotations[shadeIndex][2][:8] == 'Speech::':
                #print audioGlobals.greenIndex, len(audioGlobals.GreenShades)-1
                if audioGlobals.greenIndex >= (len(audioGlobals.GreenShades)-1):
                    audioGlobals.greenIndex = 0
                else:
                    audioGlobals.greenIndex = audioGlobals.greenIndex + 1
                #print audioGlobals.greenIndex, shadeIndex
                audioGlobals.shadesAndSpeaker.append([audioGlobals.annotations[shadeIndex][2], audioGlobals.GreenShades[audioGlobals.greenIndex]])

    # >> Call Classifier in case CSVFile not exists 
    #---------------------- 
    else:
        [flagsInd, classesAll, acc,CM] = aS.mtFileClassification(audioGlobals.wavFileName, os.path.abspath('audio/ClassifierMethods/svmModelTest'), 'svm', False)
        # declare classes
        [segs, classes] = aS.flags2segs(flagsInd, 1)
        lengthClass = len(classesAll)
        className = np.arange(lengthClass, dtype=np.float)


        for j in range(len(segs)):
            # no Annotation for Silence segments
            for i in range(len(classesAll)):
                if classes[j] == className[i] and classesAll[i] != 'Silence':
                    audioGlobals.annotations.append([segs[j][0]*1000, segs[j][1]*1000, classesAll[i]])

        # >> Write annotations in csv file
        csvFileName = audioGlobals.wavFileName.replace(".wav","_audio.csv")
        annotationFile = open(csvFileName, 'w')
        write = csv.writer(annotationFile)
        write.writerows(audioGlobals.annotations)
        annotationFile.close()


    # >> Initialize GUI 
    #----------------------
    #qApp = QtWidgets.QApplication(sys.argv)
    #aw = gui.ApplicationWindow()
    #aw.setWindowTitle("Audio")
    #aw.show()

    # >> Terminate GUI 
    #---------------------- 
    #sys.exit(qApp.exec_())
