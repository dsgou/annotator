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
import visualizeAudio as vA

import ganttChartAudio as gA

# >> Delete annotated segment in overlaping case
def deleteFromOverlap(action):
    label =  action.text()

    for element in range(len(audioGlobals.annotations)):
        #check for deletion
        if (label == audioGlobals.annotations[element][2]) and audioGlobals.xCheck >= audioGlobals.annotations[element][0] and audioGlobals.xCheck <= audioGlobals.annotations[element][1]:
            audioGlobals.annotations.remove(audioGlobals.annotations[element])
            break
    #Re-Write annotations in csv file with changes
    csvFileName =  audioGlobals.bagFile.replace(".bag","_audio.csv")  
    
    annotationFile = open(csvFileName, 'w')
    write = csv.writer(annotationFile)
    write.writerows(audioGlobals.annotations)
    annotationFile.close()
    audioGlobals.isDeleted = True
    audioGlobals.isBold = False
    #audioGlobals.counterClick = audioGlobals.counterClick - 1
    

    audioGlobals.fig.axes.clear()
    audioGlobals.fig.drawWave()
    audioGlobals.fig.drawAnnotations()
    audioGlobals.fig.draw()

    #if csv is empty -> delete file
    if not audioGlobals.annotations:
        os.remove(csvFileName)

    audioGlobals.chartFig.axes.clear()
    audioGlobals.chartFig.drawChart()
    audioGlobals.chartFig.draw()


# >> Delete Annotated Segments
#----------------------
def delete():
    tempDelete = None
    # >> Find element to delete
    for element in range(len(audioGlobals.annotations)):
        #check for deletion
        if audioGlobals.startTimeToPlay == audioGlobals.annotations[element][0] and audioGlobals.endTimeToPlay == audioGlobals.annotations[element][1]:
            tempDelete = audioGlobals.annotations[element][2]
            audioGlobals.annotations.remove(audioGlobals.annotations[element])
            break
    
    #Re-Write annotations in csv file with changes
    #----------------------
    csvFileName = audioGlobals.bagFile.replace(".bag","_audio.csv")
    annotationFile = open(csvFileName, 'w')
    write = csv.writer(annotationFile)
    write.writerows(audioGlobals.annotations)
    annotationFile.close()
    audioGlobals.isDeleted = True
    audioGlobals.isBold = False
    audioGlobals.counterClick = 1

    #if csv is empty -> delete file
    if not audioGlobals.annotations:
        os.remove(csvFileName)

    audioGlobals.fig.axes.clear()
    audioGlobals.fig.drawWave()
    audioGlobals.fig.drawAnnotations()
    audioGlobals.fig.draw()

    audioGlobals.chartFig.axes.clear()
    audioGlobals.chartFig.drawChart()
    audioGlobals.chartFig.draw()

 # >> Save current ANNOTATION
#----------------------
def saveAnnotation():
    temp = False
    annotationChange = False
    speakerExist = False

    # >> Define start-end annotation time
    startAnnotation = float(audioGlobals.startTimeToPlay)/1000.0
    endAnnotation = float(audioGlobals.endTimeToPlay)/1000.0
    iStart = np.argmin(np.abs(audioGlobals.timeArrayToPlot - startAnnotation))
    iEnd = np.argmin(np.abs(audioGlobals.timeArrayToPlot - endAnnotation))

    checkStart = startAnnotation * 1000
    checkEnd = endAnnotation * 1000

    # >> Check for change existing annotation
    for index in range(len(audioGlobals.annotations)):
        if checkStart == audioGlobals.annotations[index][0] and checkEnd == audioGlobals.annotations[index][1] and audioGlobals.text_ == audioGlobals.annotations[index][2]:
            annotationChange = True
            break
        if checkStart == audioGlobals.annotations[index][0] and checkEnd == audioGlobals.annotations[index][1]:
            audioGlobals.annotations[index][2] = audioGlobals.text_
            annotationChange = True
            for i in range(len(audioGlobals.shadesAndSpeaker)):
                if audioGlobals.text_ == audioGlobals.shadesAndSpeaker[i][0]:
                    audioGlobals.colorName = audioGlobals.shadesAndSpeaker[i][1]
                    speakerExist = True
                    break
                else:
                    speakerExist = False

            if len(audioGlobals.shadesAndSpeaker) == 0:
                speakerExist = False
                audioGlobals.colorName = audioGlobals.GreenShades[audioGlobals.greenIndex]
                audioGlobals.shadesAndSpeaker.append([audioGlobals.text_, audioGlobals.colorName])


    # >> Append new annotation
    if not annotationChange:   
        # >> List of annotations
        audioGlobals.annotations.append([audioGlobals.startTimeToPlay, audioGlobals.endTimeToPlay, audioGlobals.text_])
        for i in range(len(audioGlobals.shadesAndSpeaker)):
            if audioGlobals.text_ == audioGlobals.shadesAndSpeaker[i][0]:
                audioGlobals.colorName = audioGlobals.shadesAndSpeaker[i][1]
                speakerExist = True
                break
            else:
                speakerExist = False

        if len(audioGlobals.shadesAndSpeaker) == 0:
            audioGlobals.colorName = audioGlobals.GreenShades[audioGlobals.greenIndex]
            audioGlobals.shadesAndSpeaker.append([audioGlobals.text_, audioGlobals.colorName])

    # >> Merge audioGlobals.audioGlobals.annotations(side by side)
    for index in range(len(audioGlobals.annotations)):
        if checkEnd == audioGlobals.annotations[index][0] and audioGlobals.text_ == audioGlobals.annotations[index][2]: 
            audioGlobals.annotations[index][0] = audioGlobals.startTimeToPlay
            audioGlobals.annotations.remove(audioGlobals.annotations[index-1])
            annotationChange = True
        if checkStart == audioGlobals.annotations[index][1] and audioGlobals.text_ == audioGlobals.annotations[index][2]:
            audioGlobals.annotations[index][1] = audioGlobals.endTimeToPlay
            audioGlobals.annotations.remove(audioGlobals.annotations[index+1])
            annotationChange = True
            break
        if checkStart < audioGlobals.annotations[index][0] and checkEnd > audioGlobals.annotations[index][1] and audioGlobals.text_ == audioGlobals.annotations[index][2]:
            audioGlobals.annotations.remove(audioGlobals.annotations[index])
            annotationChange = True
            break
        if checkStart > audioGlobals.annotations[index][0] and checkStart < audioGlobals.annotations[index][1] and audioGlobals.text_ == audioGlobals.annotations[index][2]:
            for i in range(len(audioGlobals.annotations)):
                if checkStart == audioGlobals.annotations[i][0] and checkEnd == audioGlobals.annotations[i][1]:
                    audioGlobals.annotations.remove(audioGlobals.annotations[i])
                    break
            audioGlobals.annotations[index][1] = audioGlobals.endTimeToPlay
            annotationChange = True
            break
        if checkEnd > audioGlobals.annotations[index][0] and checkEnd < audioGlobals.annotations[index][1] and audioGlobals.text_ == audioGlobals.annotations[index][2]:
            for i in range(len(audioGlobals.annotations)):
                if checkStart == audioGlobals.annotations[i][0] and checkEnd == audioGlobals.annotations[i][1]:
                    audioGlobals.annotations.remove(audioGlobals.annotations[i])
                    break
            audioGlobals.annotations[index][0] = audioGlobals.startTimeToPlay
            annotationChange = True
            break

    if not speakerExist:
        if audioGlobals.greenIndex == len(audioGlobals.GreenShades):
            audioGlobals.greenIndex = 0
        else:
            audioGlobals.greenIndex = audioGlobals.greenIndex + 1
            audioGlobals.colorName = audioGlobals.GreenShades[audioGlobals.greenIndex]
            audioGlobals.shadesAndSpeaker.append([audioGlobals.text_, audioGlobals.colorName])
    

    # >> Write annotations in csv file
    csvFileName = audioGlobals.bagFile.replace(".bag","_audio.csv")
    annotationFile = open(csvFileName, 'w')
    write = csv.writer(annotationFile)
    write.writerows(audioGlobals.annotations)
    annotationFile.close()

    # >> Plot Annotated Segment 
    audioGlobals.annotationFlag = True
    audioGlobals.isBold = False
    audioGlobals.counterClick = audioGlobals.counterClick - 1
    

    audioGlobals.fig.axes.clear()
    audioGlobals.fig.drawWave()
    audioGlobals.fig.drawAnnotations()
    audioGlobals.fig.draw()

    audioGlobals.chartFig.axes.clear()
    audioGlobals.chartFig.drawChart()
    audioGlobals.chartFig.draw()