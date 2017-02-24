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
import editAnnotations as eA

annotationColors = (['Speech', 'green'],['Music','red'], ['Activity', 'magenta'],['Laugh', 'yellow'], ['Cough', '#4B0082'], ['Moan', '#800000'], ['Steps', '#FFA500'], ['TV', '#6F4E37'])

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  ONCLICK FUNCTION                                                                       #
#  count clicks on Waveform                                                               #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

def onclick(event):
    global annotationColors

    audioGlobals.tempPass = False
    flagDraw = True
    color = 'turquoise'

    if event.xdata != None:

        #Left Mouse Click -> select audio segment\
        #----------------------
        if event.button == Qt.LeftButton:
            # >> Get Clicks for Start-End Time...
            x = event.xdata
            # >> Clear axes...
            audioGlobals.fig.clear()

            # >> First Click
            if audioGlobals.counterClick == 1:
                audioGlobals.xStart = x*1000
                print 'Start time %f ms' %audioGlobals.xStart      
                audioGlobals.durationFlag = 1
                if audioGlobals.annotationFlag == True:
                    audioGlobals.tempPass = True
                    audioGlobals.annotationFlag = False
                
                audioGlobals.playerStarted = False
                
                # >> Start not out of waveform bounds
                if audioGlobals.xStart < 0:
                    audioGlobals.xStart = 0
                    print '-> Correction time %f ms' %audioGlobals.xStart

            # >> Second Click
            if audioGlobals.counterClick == 2 and audioGlobals.tempPass == False:
                if audioGlobals.isBold:
                    audioGlobals.isBold = False
                    audioGlobals.durationFlag = 0
                    audioGlobals.fig.clear()
                else:
                    audioGlobals.xEnd = x*1000
                    print 'End Time %f ms' %audioGlobals.xEnd
                    audioGlobals.durationFlag = 2
                    #check xStart < xEnd
                    if audioGlobals.xStart > audioGlobals.xEnd:
                        temp = audioGlobals.xStart
                        audioGlobals.xStart = audioGlobals.xEnd
                        audioGlobals.xEnd = temp
                        print '  '
                        print 'SWAP START-END TIME'
                        print '-------------'
                        print 'Start Time %f ms' %audioGlobals.xStart
                        print 'End Time %f ms' %audioGlobals.xEnd
                        print '-------------'
                audioGlobals.playerStarted = False


            audioGlobals.startTimeToPlay = audioGlobals.xStart
            audioGlobals.endTimeToPlay = audioGlobals.xEnd

            
            audioGlobals.counterClick = audioGlobals.counterClick + 1
            playFlag = False
            audioGlobals.fig.drawCursor(audioGlobals.xStart, audioGlobals.xEnd, color, playFlag)
            audioGlobals.fig.draw()
        else:
            #get mouse coords to specify delete widget position
            #----------------------
            audioGlobals.xPos = event.x
            audioGlobals.xCheck = event.xdata
            audioGlobals.xCheck = audioGlobals.xCheck * 1000

            audioGlobals.fig.clear()
            #Check for existing annotation
            print '=========================='
            print 'Selected segment'
            if not audioGlobals.selected:
                for index in range(len(audioGlobals.annotations)):
                    if audioGlobals.xCheck >= audioGlobals.annotations[index][0] and audioGlobals.xCheck <= audioGlobals.annotations[index][1]:
                        audioGlobals.startTimeToPlay = audioGlobals.annotations[index][0]
                        audioGlobals.endTimeToPlay = audioGlobals.annotations[index][1]
                        for colorIndex in range(len(annotationColors)):
                            if annotationColors[colorIndex][0] == audioGlobals.annotations[index][2]:
                                color = annotationColors[colorIndex][1]
            
                            elif audioGlobals.annotations[index][2][:8] == 'Speech::':
                                for shadeIndex in range(len(audioGlobals.shadesAndSpeaker)):
                                    if audioGlobals.annotations[index][2] == audioGlobals.shadesAndSpeaker[shadeIndex][0]:
                                        color = audioGlobals.shadesAndSpeaker[shadeIndex][1]

                audioGlobals.tempPass = True
                audioGlobals.durationFlag = 2
                playFlag = False
                audioGlobals.playerStarted = False
                audioGlobals.fig.drawCursor(audioGlobals.startTimeToPlay, audioGlobals.endTimeToPlay, color, playFlag)
                #fig.draw()
            else:
                audioGlobals.tempPass = True
                audioGlobals.durationFlag = 2
                audioGlobals.counterClick = audioGlobals.counterClick + 1
                playFlag = False
                audioGlobals.fig.drawCursor(audioGlobals.startTimeToPlay, audioGlobals.endTimeToPlay, color, playFlag)
                #fig.draw()
                audioGlobals.selected = False

            xS = audioGlobals.startTimeToPlay
            xE = audioGlobals.endTimeToPlay

            print 'Start time %f ms' %xS 
            print 'End time %f ms' %xE 
            print '=========================='
            #check for selected segment
            if audioGlobals.xCheck >= xS and audioGlobals.xCheck <= xE:
                audioGlobals.fig.drawBold(color)
                audioGlobals.fig.draw()
                audioGlobals.fig.annotationMenu()
                audioGlobals.fig.draw()


class Window(FigureCanvas):

    def __init__(self, parent=None, width=15, height=2, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        #self.drawWave()
        self.drawAnnotations()


        FigureCanvas.__init__(self, fig)
        #self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        clickID = fig.canvas.mpl_connect('button_press_event', onclick)
        #clickID = fig.canvas.mpl_connect('motion_notify_event', onmotion)

    def drawWave(self):
        pass

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  POP UP WINDOW                                                                          #
#  called from WAVEFORM                                                                   #
#  open new window to add new speakers                                                    #
#  click "OK" only in completed edit box                                                  #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

class MyPopup(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.setWindowTitle('Add new Speaker')
        self.main_widget = QtWidgets.QWidget(self)
        self.speakerID = QLineEdit(self)
        self.Ok = QPushButton("Ok", self)
        #self.show()

    # Draw new Speaker window
    #----------------------
    def paintEvent(self, e):
        self.speakerID.setPlaceholderText('Speaker...')
        self.speakerID.setMinimumWidth(100)
        self.speakerID.setEnabled(True)

        self.speakerID.move(90, 15)
        self.Ok.move(115, 60)

        self.speakerID.textChanged.connect(self.speakerLabel)
        self.Ok.clicked.connect(self.closeSpeaker)

        self.Ok.show()
        self.speakerID.show()

    def speakerLabel(self,text):
        audioGlobals.text_ = 'Speech::' + text 

    # Close and save new Speaker ID
    #----------------------
    def closeSpeaker(self):
        if audioGlobals.text_ != 'Add New Speaker':
            audioGlobals.text_ = 'Speech::' + self.speakerID.text()
            self.Ok.clicked.disconnect() 
            self.close()

            eA.saveAnnotation()
            audioGlobals.fig.draw()
            audioGlobals.chartFig.axes.clear()
            audioGlobals.chartFig.drawChart()
            audioGlobals.chartFig.draw()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  WAVEFORM PLOT FUNCTION                                                                 #
#  called from application window                                                         #
#  draw signal and annotations                                                            #
#  initialize deletion and choice annotation menu                                         #
#  save annotations in .csv file                                                          #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

class Waveform(Window):
    global annotationColors

    # >> CLEAR PLOTS AND RE-DRAW CHANGES
    #----------------------
    def clear(self):
        self.axes.clear()
        self.drawWave()
        self.drawAnnotations()
        self.draw()

    # >> PLOT WAVEFORM
    #----------------------
    def drawWave(self):
        self.axes.clear()
        #PLOT WAVEFORM
        self.plotStep = 100
        self.signalToPlot = audioGlobals.signal[0:-1:self.plotStep]

        self.timeArray = np.arange(0,audioGlobals.signal.shape[0] / 16000.0, 1.0/16000)
        audioGlobals.timeArrayToPlot = self.timeArray[0:-1:self.plotStep]

        #Plot Waveform Signal
        self.axes.get_xaxis().set_visible(True)
        self.axes.plot(audioGlobals.timeArrayToPlot, self.signalToPlot)
        self.axes.set_yticklabels([])
        self.axes.set_xlim([-1,audioGlobals.duration + 1])
        audioGlobals.xTicks = self.axes.get_xticks()
        self.axes.grid(True)

    # >> PLOT CURSORS AND SELECTED AREA
    #----------------------
    def drawCursor(self,start, end, c, playFlag):
        #Media player variables

        tStart = float(start)/1000.0
        tEnd = float(end)/1000.0
        iS = np.argmin(np.abs(audioGlobals.timeArrayToPlot - tStart))
        iE = np.argmin(np.abs(audioGlobals.timeArrayToPlot - tEnd))
        #Draw Start-End Cursors
        if audioGlobals.durationFlag == 2:
            self.axes.axvline(tStart,color='Navy', alpha=0.5)
            self.axes.axvline(tEnd,color='Navy', alpha=0.5)
            audioGlobals.selected = True
        elif audioGlobals.durationFlag == 1:
            self.axes.axvline(tStart,color='Navy', alpha=0.5)
            if audioGlobals.playerStarted == True:
                self.axes.axvline(tEnd,color='Navy', alpha=0.5)
        else:
            if audioGlobals.playerStarted == True:
                self.axes.axvline(tStart,color='Navy', alpha=0.5)
                self.axes.axvline(tEnd,color='Navy', alpha=0.5)
        if playFlag == False:
            self.axes.plot(audioGlobals.timeArrayToPlot[iS:iE],self.signalToPlot[iS:iE],color=c, alpha=0.65)
        elif playFlag == True:
            self.axes.plot(audioGlobals.timeArrayToPlot[iS:iE],self.signalToPlot[iS:iE],color='black')

        # >> Zero Clicks , Counters and Flags -> Release Button Function
        #----------------------
        if audioGlobals.counterClick >= 3:
            audioGlobals.xStart = 0
            audioGlobals.xEnd = 0
            audioGlobals.tempPass = False
            audioGlobals.playerStarted = False
            audioGlobals.counterClick = 1
            audioGlobals.annotationFlag = False
            audioGlobals.isDeleted = False
            audioGlobals.isBold = False

    # >> BOLD SELECTED SEGMENT
    #----------------------
    def drawBold(self, c):

        tStart = float(audioGlobals.startTimeToPlay)/1000.0
        tEnd = float(audioGlobals.endTimeToPlay)/1000.0
        iS = np.argmin(np.abs(audioGlobals.timeArrayToPlot - tStart))
        iE = np.argmin(np.abs(audioGlobals.timeArrayToPlot - tEnd))

        start = tStart * 1000
        end = tEnd *1000
        audioGlobals.text1 = ' '
        audioGlobals.text2 = ' '
        text = ' '

        #position to diplay Class-Text
        positionPlotText = (tStart+tEnd)/2

        #check for change existing annotation
        #----------------------
        for class1 in range(len(audioGlobals.annotations)):
            if start == audioGlobals.annotations[class1][0] and end == audioGlobals.annotations[class1][1]:
                audioGlobals.text1 = audioGlobals.annotations[class1][2]
                for class2 in range(len(audioGlobals.annotations)):
                    if (audioGlobals.annotations[class1][0] < audioGlobals.annotations[class2][0] and audioGlobals.annotations[class1][1] > audioGlobals.annotations[class2][1] or
                        audioGlobals.annotations[class1][0] > audioGlobals.annotations[class2][0] and audioGlobals.annotations[class1][1] < audioGlobals.annotations[class2][1] or
                        audioGlobals.annotations[class1][0] < audioGlobals.annotations[class2][0] and audioGlobals.annotations[class1][1] > audioGlobals.annotations[class2][0] or
                        audioGlobals.annotations[class1][0] < audioGlobals.annotations[class2][1] and audioGlobals.annotations[class1][1] > audioGlobals.annotations[class2][1]):
                        audioGlobals.text2 = audioGlobals.annotations[class2][2]

        #find if segment is above or not
        for class1 in range(len(audioGlobals.annotations)):
            if start == audioGlobals.annotations[class1][0] and end == audioGlobals.annotations[class1][1]:
                audioGlobals.text1 = audioGlobals.annotations[class1][2]
                for class2 in range(len(audioGlobals.annotations)):
                        if audioGlobals.annotations[class1][0] < audioGlobals.annotations[class2][0] and audioGlobals.annotations[class1][1] > audioGlobals.annotations[class2][1]:
                            if class2 < class1:
                                audioGlobals.isAbove = True
                                
        if audioGlobals.text2 != ' ':
            text = audioGlobals.text1 + ' & ' + audioGlobals.text2
            audioGlobals.overlap = True
        else:
            text = audioGlobals.text1
            audioGlobals.overlap = False

        # >> Plot Class-Text and Bold Line
        self.axes.text(positionPlotText, 3100, text,horizontalalignment='center',verticalalignment='center')
        self.axes.plot(audioGlobals.timeArrayToPlot[iS:iE],self.signalToPlot[iS:iE],color=c, linewidth=3.5) 

        #flag isBold -> test on clear if selected area is bold to return in normal plot
        audioGlobals.isBold = True
        audioGlobals.selected = False


    def annotationMenu(self):
        speakers = []
        passAppend = True

        annotation = QMenu()

        for index in range(len(audioGlobals.annotations)):
            if audioGlobals.startTimeToPlay==audioGlobals.annotations[index][0] and audioGlobals.endTimeToPlay==audioGlobals.annotations[index][1]:
                if not audioGlobals.overlap:
                    delete = annotation.addAction('Delete')
                    delete.triggered.connect(eA.delete)
                elif audioGlobals.overlap and audioGlobals.isAbove:
                    delete = annotation.addMenu('Delete')
                    delete.addAction(audioGlobals.text1)
                    delete.addAction(audioGlobals.text2)
                    delete.triggered.connect(eA.deleteFromOverlap)
                    audioGlobals.isAbove = False
                elif audioGlobals.overlap:
                    delete = annotation.addAction('Delete')
                    delete.triggered.connect(eA.delete)
                    audioGlobals.overlap = False
        self.subMenu = annotation.addMenu('Annotate')
        
        #Define Labels
        #----------------------
        for i in range(len(audioGlobals.classLabels)):
            self.subMenu.addAction(audioGlobals.classLabels[i])

        #Define Speakers
        #----------------------
        speakerMenu = self.subMenu.addMenu('Speech')
        for i in range(len(audioGlobals.annotations)):
            if audioGlobals.annotations[i][2][:8] == 'Speech::':
                remove = audioGlobals.annotations[i][2].index(':')
                remove = remove + 2
                length = len(audioGlobals.annotations[i][2])
                sub = length - remove
                if not audioGlobals.annotations[i][2][-sub:] in speakers:
                    speakers.append(audioGlobals.annotations[i][2][-sub:])
                #speakerMenu.addAction(speaker)

        for index in range(len(speakers)):
            speakerMenu.addAction(speakers[index])
        addNew = speakerMenu.addAction('Add New Speaker')

        self.subMenu.triggered.connect(self.chooseAnnotation)
        speakerMenu.triggered.connect(self.Speakers)

        annotation.exec_(self.mapToGlobal(self.pos()) + QtCore.QPoint(audioGlobals.xPos, audioGlobals.yPos))

    

    # >> Annotate Speakers
    #----------------------
    def Speakers(self, action):
        text = action.text()

        if text == 'Add New Speaker':
            self.newSpeaker()
        else:
            audioGlobals.text_ = 'Speech::' + text
            eA.saveAnnotation()
            self.draw()
            audioGlobals.chartFig.axes.clear()
            audioGlobals.chartFig.drawChart()
            audioGlobals.chartFig.draw()

    # >> Add new speaker
    #----------------------
    def newSpeaker(self):
        self.new = MyPopup()
        self.new.setGeometry(QRect(590, 800, 300, 100))
        self.new.show()

    # >> Annotate Audio Segments from Annotation Menu
    #----------------------
    def chooseAnnotation(self, action):
        global doIt

        doIt = False

        text = action.text()
        audioGlobals.text_ = text


        if text == 'Speech':
            audioGlobals.colorName = 'green'
            doIt = True

        elif text == 'Music':
            audioGlobals.colorName = 'red'
            doIt = True

        elif text == 'Activity':
            audioGlobals.colorName = 'magenta'
            doIt = True

        elif text == 'Laugh':
            audioGlobals.colorName = 'yellow'
            doIt = True
        elif text == 'Cough':
            audioGlobals.colorName = '#4B0082'
            doIt = True
        elif text == 'Moan':
            audioGlobals.colorName = '#800000'
            doIt = True
        elif text == 'Steps':
            doIt = True
        elif text =='TV':
            doIt = True
        
        if doIt:
            eA.saveAnnotation()
            self.draw()
            audioGlobals.chartFig.axes.clear()
            audioGlobals.chartFig.drawChart()
            audioGlobals.chartFig.draw()


    # >> PLOT annotations SAVED ON .CSV FILE
    #----------------------
    def drawAnnotations(self):
        self.axes.set_yticklabels([])
        self.axes.get_yaxis().set_visible(False)
        color = None
        for index in range(len(audioGlobals.annotations)):
            startAnnotation = float(audioGlobals.annotations[index][0])/1000.0
            endAnnotation = float(audioGlobals.annotations[index][1])/1000.0
            iStart = np.argmin(np.abs(audioGlobals.timeArrayToPlot - startAnnotation))
            iEnd = np.argmin(np.abs(audioGlobals.timeArrayToPlot - endAnnotation))
            # if Speech define greenshade
            if audioGlobals.annotations[index][2][:8] == 'Speech::':
                for shadeIndex in range(len(audioGlobals.shadesAndSpeaker)):
                    if audioGlobals.annotations[index][2] == audioGlobals.shadesAndSpeaker[shadeIndex][0]:
                        color = audioGlobals.shadesAndSpeaker[shadeIndex][1]
                pass
            # annotate rest of Classes
            else:
                for colorIndex in range(len(annotationColors)):
                    if annotationColors[colorIndex][0] == audioGlobals.annotations[index][2]:
                        color = annotationColors[colorIndex][1]
            self.axes.plot(audioGlobals.timeArrayToPlot[iStart:iEnd],self.signalToPlot[iStart:iEnd],color=color)

                                       
    