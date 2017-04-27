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

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  GUI FUNCTION                                                                           #
#  called from run Functions                                                              #
#  Initilize Graphical Interface widgets                                                  #
#  Enable QtMultimedia Player                                                             #
#  Enable annotation and gantt chart plots                                                #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

class ApplicationWindow(QtWidgets.QMainWindow):

    # >> QtMultimedia Signals
    #----------------------
    play = pyqtSignal()
    pause = pyqtSignal()
    stop = pyqtSignal()

    def __init__(self):

        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

        self.main_widget = QtWidgets.QWidget(self)
        audioGlobals.playerStarted = False

        #DEFINE PLAYER-PLAYLIST   
        #----------------------
        self.source = QtCore.QUrl.fromLocalFile(os.path.abspath(audioGlobals.wavFileName))
        self.content = QMediaContent(self.source)
        self.player = QMediaPlayer()
        self.playlist = QMediaPlaylist(self)
        self.playlist.addMedia(self.content)
        self.player.setPlaylist(self.playlist)

        # >> Define annotations and gantt chart 
        #---------------------- 
        self.wave = vA.Waveform()
        audioGlobals.fig = self.wave
        self.chart = gA.Chart()
        audioGlobals.chartFig = self.chart

        # >> Define player buttons 
        #---------------------- 
        playButton = QPushButton("Play")
        pauseButton = QPushButton("Pause")
        stopButton = QPushButton("Stop")

        # >> Define layouts 
        #---------------------- 
        waveLayout = QVBoxLayout()
        waveLayout.addWidget(self.wave)
        waveLayout.addWidget(self.chart)

        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setSizePolicy(QSizePolicy.Minimum,QSizePolicy.Expanding)
        waveLayout.addWidget(line)

        #Buttons layout
        buttonLayout = QVBoxLayout()
        buttonLayout.addWidget(playButton)
        buttonLayout.addWidget(pauseButton)
        buttonLayout.addWidget(stopButton)
        buttonLayout.setAlignment(Qt.AlignTop)


        # >> Specify final layout align 
        #----------------------
        layout = QHBoxLayout(self.main_widget)
        layout.addLayout(waveLayout)
        layout.addLayout(buttonLayout)
        
        # >> Define buttons connections 
        #---------------------- 
        playButton.clicked.connect(self.Play)
        pauseButton.clicked.connect(self.Pause)
        stopButton.clicked.connect(self.Stop)


        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)


    # PLAYER BUTTON FUNCTIONS

    # >> Play audio (whole signal or segment) 
    #---------------------- 
    def Play(self):

        #GET CLICKS FROM WAVEFORM
        #---------------------- 
        #Initialize connection-position ONCE
        if not audioGlobals.playerStarted:
            #10ms for changePosition -> Not Delaying
            self.player.positionChanged.connect(self.checkPositionToStop)
            self.player.setNotifyInterval(10)
            if audioGlobals.durationFlag==0:
                audioGlobals.playerStarted = True
                audioGlobals.startTimeToPlay = 0
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.duration*1000 - 10
                audioGlobals.endTimeToPlay = self.end
                audioGlobals.counterClick = 3
            elif audioGlobals.durationFlag==1:
                audioGlobals.playerStarted = True
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.duration*1000 - 10
                audioGlobals.endTimeToPlay = self.end
                audioGlobals.counterClick = 3
            elif audioGlobals.durationFlag==2:
                audioGlobals.playerStarted = True
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.endTimeToPlay
            self.player.setPosition(self.start)

        playFlag = True
        self.player.play()

    # >> Pause audio playing 
    #----------------------
    def Pause(self):
        #Not begging from self.start
        audioGlobals.playerStarted = True
        self.player.setPosition(self.time_)
        self.player.pause()

    # >> Stop audio playing 
    #----------------------
    def Stop(self):
        self.player.stop()
        #Begin again segment
        self.start = audioGlobals.startTimeToPlay
        self.player.setPosition(self.start)

    # >> Check ms in audio to stop play 
    #----------------------
    def checkPositionToStop(self):
        self.time_ = self.player.position()
        tStart = float(self.time)/1000.0
        iS = np.argmin(np.abs(audioGlobals.timeArrayToPlot - tStart))
        audioGlobals.fig.axes.plot(audioGlobals.timeArrayToPlot[iS],self.signalToPlot[iS], color = 'black', alpha=0.65)
        print self.time_
        if self.time_ >= self.end:
            self.Stop()
            self.player.setPosition(self.start)

    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()