#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import csv
import yaml
import json
import math
import time
import rosbag

import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
from termcolor import colored

#Module imports
from gui import topicBox
from gui import changeBoxId
from gui import editLabels
from gui import videoShortcuts

from audio import rosbagAudio
from audio import visualizeAudio as vA
from audio import ganttChartAudio as gA
from audio import saveAudioSegments
from audio.audioGlobals import audioGlobals
from audio.graphicalInterfaceAudio import ApplicationWindow

from video import rosbagDepth
from video import rosbagRGB
from video import rosbagVideo
from video import gantChart
from video.videoGlobals import videoGlobals
''''''''''''''''''''''''''''''''''''


global bagFile
global csvFile
global videoCSV
global headlines
global frameCounter
global boxInitialized
global BasicTopics
global delete_index
global audio_player
global depth_player
global video_player
global mainWindow

bagFile   = None
videoCSV  = None
xBoxCoord = []
frameCounter   = 0
delete_index   = -1
audio_player   = False
depth_player   = False
video_player   = False
boxInitialized = False
headlines      = ["Timestamp", "Rect_id", "Rect_x", "Rect_y", "Rect_W", "Rect_H", "Class"]


mainWindow     = None
rgbFileName    = None
depthFileName  = None


def get_bag_metadata(bag):
    topics_list = []
    info_dict = yaml.load(bag._get_yaml_info())
    topics =  info_dict['topics']
    
    for top in topics:
        topics_list.append(top["topic"])
    topics_list = sorted(set(topics_list))
    duration = info_dict['duration']
    return topics_list, duration
    

class VideoWidgetSurface(QAbstractVideoSurface):

    def __init__(self, widget, parent=None):
        super(VideoWidgetSurface, self).__init__(parent)
        self.widget = widget
        self.imageFormat = QImage.Format_Invalid

    def supportedPixelFormats(self, handleType=QAbstractVideoBuffer.NoHandle):
        formats = [QVideoFrame.PixelFormat()]
        if (handleType == QAbstractVideoBuffer.NoHandle):
            for f in [QVideoFrame.Format_RGB32, QVideoFrame.Format_ARGB32, QVideoFrame.Format_ARGB32_Premultiplied, QVideoFrame.Format_RGB565, QVideoFrame.Format_RGB555,QVideoFrame.Format_BGR24,QVideoFrame.Format_RGB24]:
                formats.append(f)
        return formats

    def isFormatSupported(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        _bool = False
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty() and _format.handleType() == QAbstractVideoBuffer.NoHandle):
            _bool = True
        return _bool

    def start(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty()):
            self.imageFormat = imageFormat
            self.imageSize = size
            self.sourceRect = _format.viewport()
            QAbstractVideoSurface.start(self, _format)
            self.widget.updateGeometry()
            self.updateVideoRect()
            return True
        else:
            return False

    def stop(self):
        self.currentFrame = QVideoFrame()
        self.targetRect = QRect()
        QAbstractVideoSurface.stop(self)

        self.widget.update()

    def present(self, frame):
        global frameCounter
        if (self.surfaceFormat().pixelFormat() != frame.pixelFormat() or self.surfaceFormat().frameSize() != frame.size()):
            self.setError(QAbstractVideoSurface.IncorrectFormatError)
            self.stop()
            return False
        else:
            if frameCounter < player.message_count - 1:
                frameCounter += 1
            self.currentFrame = frame
            self.widget.repaint(self.targetRect)
            return True

    def videoRect(self):
        return self.targetRect

    def updateVideoRect(self):
        size = self.surfaceFormat().sizeHint()
        size.scale(self.widget.size().boundedTo(size), Qt.KeepAspectRatio)
        self.targetRect = QRect(QPoint(0, 0), size);
        self.targetRect.moveCenter(self.widget.rect().center())
    
    def paint(self, painter):
        if (self.currentFrame.map(QAbstractVideoBuffer.ReadOnly)):
            oldTransform = painter.transform()
            if (self.surfaceFormat().scanLineDirection() == QVideoSurfaceFormat.BottomToTop):
                painter.scale(1, -1);
                painter.translate(0, -self.widget.height())

            image = QImage(self.currentFrame.bits(),
                    self.currentFrame.width(),
                    self.currentFrame.height(),
                    self.currentFrame.bytesPerLine(),
                    self.imageFormat
            )
            painter.drawImage(self.targetRect, image, self.sourceRect)
            painter.setTransform(oldTransform)
            self.currentFrame.unmap()
           
            
class VideoWidget(QWidget):

    def __init__(self, parent=None):
        super(VideoWidget, self).__init__(parent)
        self.setAutoFillBackground(False)
        self.setAttribute(Qt.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        palette = self.palette()
        palette.setColor(QPalette.Background, Qt.black)
        self.setPalette(palette)
        self.setSizePolicy(QSizePolicy.MinimumExpanding ,
        QSizePolicy.MinimumExpanding)
        self.surface = VideoWidgetSurface(self)
        self.vanishBox = False
        self.context_menu = False
        self.enableWriteBox = False
        self.annotEnabled = False
        self.annotClass = 'Clear'
        self.deleteEnabled = False
        self.buttonLabels = []
        self.addEventLabels = []
        self.stopEventLabels = []
        self.drag_start = None
        self.index = None
        self.moved = False
        
    def videoSurface(self):
        return self.surface

    #Shows the right click menu
    def contextMenuEvent(self, event):
        global gantChart
        global frameCounter
        global framerate
        global delete_index
        
        self.addEventLabels = []
        box_id = None
        if len(player.videobox) > 0:
            if event.reason() == QContextMenuEvent.Mouse:
                posX = event.pos().x()
                posY = event.pos().y()
                
                index = -1
                for i in xrange(len(player.videobox[frameCounter].box_id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if posX > x and posX < (x+w) and posY > y and posY < (y+h):
                        index = i
                
                #If the mouse click is inside a box
                if index != -1:          
                    self.context_menu = True
                    menu = QMenu(self)

                    for i in videoGlobals.classLabels:
                        self.buttonLabels.append(menu.addAction(i))
                    
                    menu.addSeparator()
                    addEvent = menu.addMenu('Add Event')
                    stopEvent = menu.addMenu('Stop Event')
                    delete = menu.addMenu('Delete')
                    deleteBox = delete.addAction('Delete Box')
                    deleteAllBoxes = delete.addAction('Delete All Boxes')
                    deleteFrom = delete.addAction('Delete From Here')
                    deleteTo = delete.addAction('Delete To')
                    
                    if(delete_index == -1):
                        deleteTo.setEnabled(False)
                    stopEvent.setEnabled(False)
                    self.stopEventLabels = []
                    self.checkStopEventMenu = []
                    
                    #Initiate add Event menu
                    for label in videoGlobals.highLabels:
                        self.addEventLabels.append(addEvent.addAction(label))
                    changeId = menu.addAction('Change Id')
                    
                    #Show only annotated high classes of the box
                    if len(player.videobox[frameCounter].annotation) > 0:
                        for annot in player.videobox[frameCounter].annotation[index]:
                            if annot in videoGlobals.highLabels and annot not in self.checkStopEventMenu:
                                self.checkStopEventMenu.append(annot)
                                self.stopEventLabels.append(stopEvent.addAction(annot))
                                stopEvent.setEnabled(True)
                    action = menu.exec_(self.mapToGlobal(event.pos()))

                    #Check which submenu clicked
                    if action is not None:
                        repaint = False
                        box_id = player.videobox[frameCounter].box_id[index]
                        
                        if action.parent() == addEvent:
                            for i, key in enumerate(self.addEventLabels):
                                if action == key:
                                    self.annotClass = videoGlobals.highLabels[i]
                                    self.annotEnabled = True
                        elif action.parent() == stopEvent:
                            for i, key in enumerate(self.stopEventLabels):
                                if action == key:
                                    for j in xrange(frameCounter, len(player.videobox)):
                                        player.videobox[j].removeEvent(box_id, self.stopEventLabels[i].text())
                                    repaint = True
                        elif action.parent() == delete:
                            if action == deleteBox:
                                player.videobox[frameCounter].removeSpecBox(index)
                            elif action ==  deleteAllBoxes:
                                player.videobox[frameCounter].removeAllBox()
                            elif action ==  deleteFrom:
                                delete_index = frameCounter
                                deleteTo.setEnabled(True)
                            elif action == deleteTo:
                                if delete_index > 0:
                                    for i in xrange(delete_index, frameCounter + 1):
                                        player.videobox[i].removeAllBox()
                                    delete_index = -1
                            repaint = True
                        else:
                            for i,key in enumerate(self.buttonLabels):
                                if action == key:
                                    self.annotClass = videoGlobals.classLabels[i]
                                    self.annotEnabled = True
                            if action == changeId:
                                #Call the textbox
                                self.newBoxId = changeBoxId.changeBoxId(player.videobox, index, frameCounter, framerate, gantChart)
                                self.newBoxId.setGeometry(QRect(500, 100, 250, 100))
                                self.newBoxId.show()
                            
                        if self.annotEnabled:
                            for i in xrange(frameCounter, len(player.videobox)):
                                player.videobox[i].changeClass(box_id, str(self.annotClass))
                            self.annotEnabled = False
                            repaint = True
                            
                        if(repaint):        
                            self.repaint()
                            gantChart.axes.clear()
                            gantChart.drawChart(player.videobox, framerate)
                            gantChart.draw()
                
                self.buttonLabels = []
                self.context_menu = False
                
    def sizeHint(self):
        return self.surface.surfaceFormat().sizeHint()

    #Shows the video and bound boxes on it
    def paintEvent(self, event):
        global frameCounter
        global timeId

        painter      = QPainter(self)
        rectPainter  = QPainter()
        boxIdPainter = QPainter()
        
       
        if (self.surface.isActive()):
            self.surface.paint(painter)
        else:
            painter.fillRect(event.rect(), self.palette().window())
        
        if len(player.videobox) > 0 and frameCounter < len(player.videobox):
            for i in xrange(len(player.videobox[frameCounter].box_id)):
                if player.videobox[frameCounter].box_id[i] != -1:
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                        rectPainter.begin(self)    
                    rectPainter.setRenderHint(QPainter.Antialiasing)    
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_id[i]))
                    boxIdPainter.end()
            

                        
        if self.moved and not event.rect().size().__eq__(self.surface.surfaceFormat().sizeHint()):
            if not rectPainter.isActive():
                rectPainter.begin(self)    
            rectPainter.setRenderHint(QPainter.Antialiasing)    
            rectPainter.setPen(QColor(0,0,255))
            rectPainter.drawRect(event.rect())
            rectPainter.end()
            
        
    #Mouse callback handling Boxes
    def mousePressEvent(self,event):
        if len(player.videobox) > 0:
            if QMouseEvent.button(event) == Qt.LeftButton and self.context_menu is False:
                QPoint.pos1 = QMouseEvent.pos(event)
                #Check the mouse event is inside a box to initiate drag n drop
                if len(player.videobox[frameCounter].box_id) > 0:
                    for i in xrange(len(player.videobox[frameCounter].box_id)):
                        x,y,w,h = player.videobox[frameCounter].box_Param[i]
                        if (event.pos().x() >= x) and (event.pos().x() <= x + w) and (event.pos().y() >= y) and (event.pos().y() <= y + h):
                            self.index = i
                            self.drag_start = (event.pos().x(), event.pos().y())
                            break
          
    def mouseMoveEvent(self, event):
        if len(player.videobox) > 0:
            if event.buttons() == Qt.LeftButton:
                if self.index is not None:
                    x,y,w,h =  player.videobox[frameCounter].box_Param[self.index]
                    st_x, st_y = self.drag_start
                    player.videobox[frameCounter].box_Param[self.index] =  event.pos().x() - (st_x - x), event.pos().y() - (st_y - y), w, h
                    self.drag_start = (event.pos().x(), event.pos().y())
                    self.repaint()
                else:
                    if QPoint.pos1.x() < event.pos().x():
                        x = QPoint.pos1.x()
                    else:
                        x = event.pos().x()
                    if QPoint.pos1.y() < event.pos().y():
                        y = QPoint.pos1.y()
                    else:
                        y = event.pos().y()
                    w = abs(event.pos().x() - QPoint.pos1.x())
                    h = abs(event.pos().y() - QPoint.pos1.y())    
                    rect = QRect(x,y,w,h)
                    self.repaint()
                    self.repaint(rect)
                self.moved = True
        
    def mouseReleaseEvent(self, event):
        if len(player.videobox) > 0:
            if QMouseEvent.button(event) == Qt.LeftButton:
                if self.moved:
                    if self.index is not None:
                        x,y,w,h =  player.videobox[frameCounter].box_Param[self.index]
                        st_x, st_y = self.drag_start
                        player.videobox[frameCounter].box_Param[self.index] =  event.pos().x() - (st_x - x), event.pos().y() - (st_y - y), w, h
                    else:
                        if QPoint.pos1.x() < event.pos().x():
                            x = QPoint.pos1.x()
                        else:
                            x = event.pos().x()
                        if QPoint.pos1.y() < event.pos().y():
                            y = QPoint.pos1.y()
                        else:
                            y = event.pos().y()
                        w = abs(event.pos().x() - QPoint.pos1.x())
                        h = abs(event.pos().y() - QPoint.pos1.y())
                        timeId = player.videobox[frameCounter].timestamp
                        player.videobox[frameCounter].addBox(timeId, None, [x,y,w,h], ['Clear'], [])
                    self.repaint()
            self.moved = False
            self.drag_start = None
            self.index = None
        
    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self.surface.updateVideoRect()

    def getColorBox(self,action):
        
        for label in action:
            if label in videoGlobals.classLabels:
                color = label
                return videoGlobals.annotationColors[videoGlobals.classLabels.index(label) % len(videoGlobals.annotationColors)]
            elif label == 'Clear':
                color = 'Clear'
                return '#0000FF'
            elif label in videoGlobals.highLabels:
                pass

        if action in videoGlobals.classLabels:
            for index,key in enumerate(videoGlobals.classLabels):
                if action == key:
                    return videoGlobals.annotationColors[index % len(videoGlobals.annotationColors)]
                elif action == 'Clear':
                    return '#0000FF'
        else:
            for index,key in enumerate(player.videobox[frameCounter].annotation):
                if key in videoGlobals.classLabels:
                    return videoGlobals.annotationColors[classLabels.index(key) % len(videoGlobals.annotationColors)]
                elif key == 'Clear':
                    return '#0000FF'


class VideoPlayer(QWidget):
    
    def __init__(self, parent=None):
        global gantChart
        global Topics
        
        super(VideoPlayer, self).__init__(parent)
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        
        #Parse json file
        videoGlobals.classLabels, videoGlobals.highLabels, videoGlobals.annotationColors, videoGlobals.eventColors = self.parseJson()
        
        Topics              = None
        self.time_          = 0
        self.time_dif       = 0
        self.duration       = 0
        self.message_count  = 0
        self.videobox       = []
        self.box_buffer     = []
        self.metric_buffer  = []
        
        #Audio variables
        self.player = QMediaPlayer()
        self.playlist = QMediaPlaylist(self)
        self.playFlag = False
        
        self.topic_window = topicBox.TopicBox()
        
        # >> DEFINE WIDGETS OCJECTS
        # >> VIDEO - DEPTH - AUDIO - GANTT CHART
        #----------------------
        self.videoWidget = VideoWidget()
        self.videoWidget.setFixedSize(640, 480)

        #Video buttons
        videoLayout = self.createVideoButtons()
                
        #Video Gantt Chart
        self.gantt = gantChart.gantShow()
        gantChart = self.gantt
        gantChart.axes.get_xaxis().set_visible(False)
        gantChart.setFixedSize(1300, 90)
        
        #Create Slider
        self.createSlider()
        
        self.controlEnabled = False

        #Specify video layout align
        laserAndVideoLayout = QHBoxLayout()
        laserAndVideoLayout.addLayout(videoLayout)

        #Audio Player buttons
        buttonLayoutAudio = self.createAudioButtons()
        waveLayout = self.createAudio()
        
        
        self.mainLayout = QVBoxLayout()
        self.mainLayout.addLayout(laserAndVideoLayout)
        self.mainLayout.addWidget(self.positionSlider)
        self.mainLayout.addWidget(self.gantt)
        self.mainLayout.addLayout(waveLayout)
        self.mainLayout.addLayout(buttonLayoutAudio)

        self.setLayout(self.mainLayout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)

    def createSlider(self):
        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setMinimum(0)
        self.positionSlider.setMaximum(self.duration)
        self.positionSlider.setTickInterval(1)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        #add label to slider about elapsed time
        self.label_tmp = '<b><FONT SIZE=3>{}</b>'
        self.timelabel = QLabel(self.label_tmp.format('Time: ' + str(self.duration)))


        self.label = QHBoxLayout()
        self.label.addWidget(self.timelabel)
        self.label.setAlignment(Qt.AlignRight)
        
    def createVideoButtons(self):
        
        verticalLine 	=  QFrame()
        verticalLine.setFrameStyle(QFrame.VLine)
        verticalLine.setSizePolicy(QSizePolicy.Minimum,QSizePolicy.Expanding)
        
        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setShortcut(QKeySequence(Qt.Key_Space))
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        # >> radio button for Depth or RGB
        #----------------------
        self.rgbButton = QRadioButton("RGB")
        self.rgbButton.setChecked(True)
        self.rgbButton.toggled.connect(self.rgbVideo)
        self.rgbButton.setEnabled(False)

        self.depthButton = QRadioButton("Depth")
        self.depthButton.toggled.connect(self.depth)
        self.depthButton.setEnabled(False)
        
        self.previousButton = QPushButton()
        self.previousButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSeekBackward))
        self.previousButton.setShortcut(QKeySequence(Qt.ALT + Qt.Key_A))
        self.previousButton.clicked.connect(self.previousFrame)
        
        self.nextButton = QPushButton()
        self.nextButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSeekForward))
        self.nextButton.setShortcut(QKeySequence(Qt.ALT + Qt.Key_D))
        self.nextButton.clicked.connect(self.nextFrame)
        
        
        
        
        
        self.controlLayout = QHBoxLayout()
        self.controlLayout.addWidget(self.playButton)
        self.controlLayout.addWidget(self.previousButton)
        self.controlLayout.addWidget(self.nextButton)
        self.controlLayout.addWidget(self.rgbButton)
        self.controlLayout.addWidget(self.depthButton)
        self.controlLayout.setAlignment(Qt.AlignLeft)
        videoLayout = QVBoxLayout()
        videoLayout.addWidget(self.videoWidget)
        videoLayout.addLayout(self.controlLayout)
        
        return videoLayout
        
    def pauseMedia(self):
        self.mediaPlayer.pause()
        self.Pause()

    #VIDEO SWITCH RGB <-> Depth
    def rgbVideo(self, enabled):
        global rgbFileName
        global audio_player
        global depth_player
        global video_player
        
        if enabled:
            self. depthEnable = False
            self.rgbEnable = True
            position = self.mediaPlayer.position()
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(rgbFileName))))
            self.mediaPlayer.setPosition(position)
            self.mediaPlayer.play()
            if audio_player:
                self.player.setPosition(position)
                self.audioPlay()
            self.playButton.setEnabled(True)

    def depth(self, enabled):
        global depthFileName
        global audio_player
        global depth_player
        global video_player

        if enabled:
            self.rgbEnable = False
            self.depthEnable = True
            position = self.mediaPlayer.position()
            if depth_player:
                self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(depthFileName))))
                self.mediaPlayer.setPosition(position)
                self.mediaPlayer.play()
            if audio_player:
                self.player.setPosition(position)
                self.audioPlay()
            self.playButton.setEnabled(True)
    
    def previousFrame(self):
        global frameCounter
        if frameCounter > 0:
            frameCounter -= 2
            pos = round(((frameCounter ) * (self.duration * 1000)) / self.message_count)
            self.mediaPlayer.setPosition(pos) 
        
    def nextFrame(self):
        global frameCounter
    
        if frameCounter < self.message_count:
            pos = round(((frameCounter ) * (self.duration * 1000)) / self.message_count)
            self.mediaPlayer.setPosition(pos) 
        
    # AUDIO PLAYER BUTTON FUNCTIONS
    def createAudio(self):
        #Define Audio annotations and gantt chart
        self.wave = vA.Waveform()
        audioGlobals.fig = self.wave
        self.wave.axes.get_xaxis().set_visible(False)
        self.wave.draw()
        self.wave.setFixedSize(1300, 175)
        
        self.audioChart = gA.Chart()
        audioGlobals.chartFig = self.audioChart
        self.audioChart.setFixedSize(1300, 90)
        
        #Audio layouts
        waveLayout = QVBoxLayout()
        waveLayout.addWidget(self.wave)
        waveLayout.addWidget(self.audioChart)
        
        return waveLayout
        
    def createAudioButtons(self):
        self.playButtonAudio = QPushButton()
        self.stopButtonAudio = QPushButton()

        self.playButtonAudio.clicked.connect(self.audioPlay)
        self.stopButtonAudio.clicked.connect(self.audioStop)
        
        self.playButtonAudio.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.stopButtonAudio.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
        
        buttonLayoutAudio = QHBoxLayout()
        buttonLayoutAudio.addWidget(self.playButtonAudio)
        buttonLayoutAudio.addWidget(self.stopButtonAudio)
        buttonLayoutAudio.setAlignment(Qt.AlignLeft)
        
        return buttonLayoutAudio
       
    #Play audio (whole signal or segment)
    def audioPlay(self):

        #GET CLICKS FROM WAVEFORM
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
            
       
        if self.playFlag:
            self.playFlag = False
            audioGlobals.playerStarted = True
            self.player.setPosition(self.time_)
            self.player.pause()
            self.playButtonAudio.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        else:
            self.playFlag = True
            self.playButtonAudio.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
            self.player.play()

    #Stop audio playing
    def audioStop(self):
        self.player.stop()
        #Begin again segment
        self.start = audioGlobals.startTimeToPlay
        self.player.setPosition(self.start)

    #Check ms in audio to stop play
    def checkPositionToStop(self):
        self.time_ = self.player.position()
        #self.positionSlider.setValue(self.time_/1000)
        if self.time_ >= self.end:
            self.audioStop()
            self.player.setPosition(self.start)

    def videoPosition(self):
        self.videoTime = self.mediaPlayer.position()

    def openFile(self):
        global framerate
        global bagFile
        global depthFileName
        global rgbFileName
        global Topics
        global audio_player
        global depth_player
        global video_player
        framerate = 0
               
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(),"(*.bag *.avi *.mp4)")
        # create a messsage box for get or load data info
        if fileName:
	    self.videobox = []
	    if fileName.split('.')[1] == 'bag':
		bagFile = fileName
		try:
		    bag = rosbag.Bag(fileName)
		    Topics, self.duration = get_bag_metadata(bag)
		    #Show window to select topics
		    self.topic_window.show_topics(Topics)
		except:
		    self.errorMessages(0)
                
                #Audio Handling
		if self.topic_window.temp_topics[0][1] != 'Choose Topic':
                    try:
                        audio_player = True
                        audioGlobals.annotations = []
                        rosbagAudio.runMain(bag, str(fileName))
                    except:
                        self.errorMessages(6)
                    
                    #DEFINE PLAYER-PLAYLIST
                    #----------------------
                    self.source = QUrl.fromLocalFile(os.path.abspath(audioGlobals.wavFileName))
                    self.content = QMediaContent(self.source)
                    self.playlist.addMedia(self.content)
                    self.player.setPlaylist(self.playlist)

                    self.wave.drawWave()
                    self.wave.drawAnnotations()
                    self.wave.draw()
                    self.audioChart.drawChart()
                    self.audioChart.draw()
            
                #Depth Handling
		if self.topic_window.temp_topics[1][1] != 'Choose Topic':
                    depth_player = True
                    depthFileName = fileName.replace(".bag","_DEPTH.avi")
                    
                    try:
                        (self.message_count, compressed, framerate) = rosbagVideo.buffer_video_metadata(bag, self.topic_window.temp_topics[1][1])
                        rosbagDepth.write_depth_video(bag, depthFileName, self.topic_window.temp_topics[1][1])
                    except:
                        self.errorMessages(7)
                
                #RGB Handling
                if self.topic_window.temp_topics[2][1] != 'Choose Topic':
                    try:
			video_player = True
			rgbFileName = fileName.replace(".bag","_RGB.avi")
			(self.message_count, compressed, framerate) = rosbagVideo.buffer_video_metadata(bag, self.topic_window.temp_topics[2][1])

			if not os.path.isfile(rgbFileName):
				#Get bag video metadata
				print(colored('Getting rgb data from ROS', 'green'))
				image_buffer = rosbagRGB.buffer_rgb_data(bag, self.topic_window.temp_topics[2][1], compressed)
				if not image_buffer:
					raise Exception(8)
				result  = rosbagRGB.write_rgb_video(rgbFileName, image_buffer, framerate)
				if not result:
					raise Exception(2)
			
			(self.duration, framerate, self.message_count) =  rosbagRGB.get_metadata(rgbFileName)
			
			# just fill time buffer in case that video exists
			start_time = None
			for topic, msg, t in bag.read_messages(topics=[self.topic_window.temp_topics[2][1]]):
				if not start_time:
					start_time = t.to_sec()
				time = t.to_sec() - start_time
				self.videobox.append(boundBox(time))
				
			if self.rgbButton:
				self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(rgbFileName))))
				self.playButton.setEnabled(True)
			elif self.depthButton:
				self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(depthFileName))))
				self.playButton.setEnabled(True) 
                            
                        
                    except Exception as e:
                        print(e)
                        self.errorMessages(e[0])	
                
            else:
                video_player = True
                self.duration, framerate, self.message_count  =  rosbagRGB.get_metadata(fileName)
                self.videobox = [boundBox(count/framerate) for count in xrange(int(self.message_count))] 
                
                if self.rgbButton:
                    self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(fileName))))
                    self.playButton.setEnabled(True)
                rgbFileName = fileName
            gantChart.axes.clear()
            gantChart.drawChart(player.videobox, framerate)
            gantChart.draw()
            mainWindow.setWindowTitle(fileName)    
        self.setWindowTitle(fileName + ' -> Annotation')
     
    
    #Open CSV file
    def openCsv(self):
        global framerate
        global rgbFileName
        global bagFile
        global videoCSV
        global headlines
        self.box_buffer = []
        
        if rgbFileName is not None:
            
            # OPEN VIDEO - DEPTH - AUDIO
            fileName,_ =  QFileDialog.getOpenFileName(self, "Open Csv ", os.path.dirname(os.path.abspath(rgbFileName)),"(*.csv)")
            if fileName:
                videoCSV = fileName
                self.videobox = [boundBox(count) for count in xrange(int(self.message_count))]
                headlines, box_buff, box_action, features = rosbagRGB.buffer_video_csv(fileName)
                if not (box_buff):
                    self.errorMessages(1)
                else:
                    self.box_buffer = [list(elem) for elem in box_buff]
                    
                    #Frame counter initialize
                    timestamp = None
                    counter = 0
                    self.box_actionBuffer = [key for key in box_action]
                    self.features = [key for key in features]
                    a = 0
                    for i, key in enumerate(self.box_buffer):
                        if timestamp is not None:
                            if timestamp != key[0]:
                                counter += 1
                        self.videobox[counter].addBox(key[0], key[1], key[2:], self.box_actionBuffer[i], features[i])
                        timestamp  = key[0]
                              
                    gantChart.axes.clear()
                    gantChart.drawChart(self.videobox, framerate)
                    gantChart.draw()
        else:
            self.errorMessages(10)
	
    #Writes the boxes to csv
    def writeCSV(self):
        global headlines
        global rgbFileName
        global video_player
        if video_player:
            
            csvFileName = rgbFileName.replace(rgbFileName.split(".")[-1],"csv")
            with open(csvFileName, 'w') as file:
                csv_writer = csv.writer(file, delimiter='\t')
                csv_writer.writerow(headlines)
                for i in xrange(0, len(self.videobox)):
                    box = self.videobox[i]
                    if len(box.box_id) > 0:
                        for j in xrange(0, len(box.box_id)):
                            master = []
                            append = master.append
                            if box.box_id[j] != -1:
                                append(box.timestamp)
                                append(box.box_id[j])
                                for param in box.box_Param[j][::]:
                                    append(param)
                                for param in box.features[j][::]:
                                    append(param)
                                append(box.annotation[j])    
                                
                                csv_writer.writerow(master)
                            else:
                                csv_writer.writerow([box.timestamp])
                    else:
						csv_writer.writerow([box.timestamp])
                    
                print ("Csv written at: ", csvFileName) 
                
                	
    def errorMessages(self, index):
        msgBox = QMessageBox()
        msgBox.setIcon(msgBox.Warning)
        if index == 0:
            msgBox.setWindowTitle("Open rosbag")
            msgBox.setText("Could not open rosbag")
        elif index == 1:
            msgBox.setWindowTitle("Open CSV")
            msgBox.setText("Could not process CSV file")
        elif index == 2:
            msgBox.setWindowTitle("Open rosbag")
            msgBox.setIcon(msgBox.Critical)
            msgBox.setText("Could not write video")
        elif index == 3:
            msgBox.setText("Error: Json file path error")
        elif index == 4:
            msgBox.setText("Not integer type")
        elif index == 5:
            msgBox.setText("Box id already given")
        elif index == 6:
            msgBox.setWindowTitle("Open rosbag")
            msgBox.setText("Incorrect Audio Topic")
        elif index == 7:
            msgBox.setWindowTitle("Open rosbag")
            msgBox.setText("Incorrect Depth Topic")
        elif index == 8:
            msgBox.setWindowTitle("Open rosbag")
            msgBox.setText("Incorrect RGB Topic")
        elif index == 10:
            msgBox.setWindowTitle("Open CSV")
            msgBox.setText("You must select a rosbag first")

        msgBox.resize(100,40)
        msgBox.exec_()

    def play(self):
        global frameCounter
        global audio_player
        global depth_player
        global video_player
        
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.videoPosition()
            self.mediaPlayer.pause()
            if audio_player:
                self.audioPause()
            self.time_ = self.positionSlider

        else:
            self.time_ = self.mediaPlayer.position()
            if audio_player:
                self.player.setPosition(self.time_)
                self.end = audioGlobals.duration*1000 - 10
                self.audioPlay()
            if video_player:
                self.mediaPlayer.play()

        # >> Get slider position for bound box
        posSlider = self.positionSlider.value()
        #self.tickLabel.setAlignment(posSlider)
        frameCounter = int(round((self.message_count * posSlider)/(self.duration * 1000)))


    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        time = "{0:.2f}".format(float(position)/1000)
        self.positionSlider.setValue(position)
        self.positionSlider.setToolTip(str(time) + ' sec')
        self.timelabel.setText(self.label_tmp.format('Time: ' + str(time) + '/ ' + str("{0:.2f}".format(self.duration)) + ' sec'))

    def keyPressEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = True

    def keyReleaseEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = False

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    def setPosition(self, position):
        global frameCounter
        global audio_player
        global depth_player
        global video_player
        
        frameCounter = int(round(self.message_count * position/(self.duration * 1000)))
        if frameCounter >= self.message_count:
            frameCounter = self.message_count - 1 
        if video_player or depth_player:
            self.mediaPlayer.setPosition(position)
        if audio_player:
            self.player.setPosition(position)

      
   
    def closeEvent(self, event):
        self.writeCSV()
    
    def parseJson(self):
        json_basicLabel = []
        json_highLabel = []
        json_annotationColors = []
        json_eventColors = []

        with open("labels.json") as json_file:
                json_data = json.load(json_file)
                json_label = []
                for i in json_data['basiclabels'] :
                    json_basicLabel.append(i)
                for i in json_data['highlevellabels']:
                    json_highLabel.append(i)
                for i in json_data['annotationColors'] :
                    json_annotationColors.append(i)
                for i in json_data['eventColors']:
                    json_eventColors.append(i)
        return json_basicLabel,json_highLabel, json_annotationColors, json_eventColors

    
#Holds the bound box parameters
class boundBox():
	
    def __init__(self, time):
        self.timestamp  = time
        self.box_id     = []
        self.box_Param  = []
        self.features   = []
        self.annotation = []

    def addBox(self, time, box_id, params, classify, features):
        self.timestamp = time
        
        boxNumber = -1
        if box_id is None:
            #If id already in the list then give the next id
            for i in xrange(len(self.box_id)):
                if(i != self.box_id[i]):
                    boxNumber = i
            if boxNumber == -1:
                boxNumber = len(player.videobox[frameCounter].box_id)
        else:
            boxNumber = box_id
        self.box_id.append(boxNumber)
        self.box_Param.append(params[::])
        self.annotation.append(classify)
        self.features.append(features)

    def removeAllBox(self):
        self.box_id[:]     = []
        self.features[:]   = []
        self.box_Param[:]  = []
        self.annotation[:] = []

    def removeSpecBox(self, index):
        self.box_id.pop(index)
        self.features.pop(index)
        self.box_Param.pop(index)
        self.annotation.pop(index)
        
    #Handles the annotation for basic and high level classes
    def changeClass(self, boxid, classify):
        if boxid in self.box_id:
            if classify in videoGlobals.classLabels:
                self.annotation[boxid][0] = classify
            elif classify not in self.annotation[self.box_id.index(boxid)]:
                self.annotation[self.box_id.index(boxid)].append(classify)

    #Remove high level events
    def removeEvent(self, boxid, action):
        if boxid in self.box_id:
            #boxid is the index of boxes
            for key in self.annotation[self.box_id.index(boxid)]:
                if action == key:
                    self.annotation[self.box_id.index(boxid)].remove(key)

    def copy(self, other):
        self.box_id = []
        self.features   = []
        self.box_Param  = []
        self.annotation = []
        
        for i in other.box_id:
            self.box_id.append(i)
        for i in other.box_Param:
            self.box_Param.append(i)
        for i in other.annotation:
            self.annotation.append(i)
        for i in other.features:
            self.features.append(i)

class MainWindow(QMainWindow):
    
    def __init__(self, player):
        super(MainWindow, self).__init__()
        self.setCentralWidget(player)
        self.createActions()
        self.fileMenu = self.menuBar().addMenu("&File")
        self.fileMenu.addAction(self.openBagAct)
        self.fileMenu.addAction(self.openCsvAct)
        self.fileMenu.addAction(self.saveCsvAct)
        self.fileMenu.addAction(self.quitAct)
        
        self.editMenu = self.menuBar().addMenu("&Video")
        self.editMenu.addAction(self.editLabelsAct)
        self.editMenu.addAction(self.deleteAct)
        self.editMenu.addAction(self.copyAct)
        self.editMenu.addAction(self.shotcutAct)
        
        
    def createActions(self):
        self.openBagAct = QAction("&Open rosbag", self, shortcut="Ctrl+B",
            statusTip="Open rosbag", triggered=self.openBag)
        self.openCsvAct = QAction("&Open video csv", self, shortcut="Ctrl+V",
            statusTip="Open csv", triggered=self.openCSV)
        self.saveCsvAct = QAction("&Save video csv", self, shortcut="Ctrl+S",
            statusTip="Save csv", triggered=self.saveCSV)
        self.quitAct = QAction("&Quit", self, shortcut="Ctrl+Q",
            statusTip="Quit", triggered=self.closeEvent)
            
        self.editLabelsAct = QAction("Edit Labels", self,
            statusTip="Edit Labels", triggered=self.edit_labels)
        self.deleteAct = QAction("Delete All Boxes", self, shortcut=Qt.ALT + Qt.Key_R,
            statusTip="Delete All Boxes", triggered=self.deleteEvent)
        self.copyAct = QAction("Copy Latest Boxes", self, shortcut=Qt.ALT + Qt.Key_E,
            statusTip="Copy Latest Boxes", triggered=self.copyPrevious)
            
        self.shotcutAct = QAction("Shortcuts", self, statusTip="Shortcut information",
            triggered=self.shortcuts)
        
    def openBag(self):
        player.openFile()
        
    def openCSV(self):
        player.openCsv()
        
    def saveCSV(self):
        player.writeCSV()
     
    def close(self):
        sys.exit(app) 
        
    def saveAndClose(self):
        player.writeCSV()
        sys.exit(app) 
    
    def closeEvent(self, event):
        global bagFile
        global videoCSV
        if bagFile and videoCSV:
            msgBox = QMessageBox()
            msgBox.setIcon(msgBox.Warning)
            msgBox.setWindowTitle("Quit")
            msgBox.setText("<b><font size=\"5\">You may have unfinished work")
            msgBox.setInformativeText("Do you want to save before quitting?")
            yesButton = QPushButton()
            yesButton.setText("Yes")
            yesButton.clicked.connect(self.saveAndClose)
            msgBox.addButton(yesButton, QMessageBox.YesRole)
            
            noButton = QPushButton()
            noButton.setText("No")
            noButton.clicked.connect(self.close)
            msgBox.addButton(noButton, QMessageBox.NoRole)
            
            cancelButton = QPushButton()
            cancelButton.setText("Cancel")
            msgBox.addButton(cancelButton, QMessageBox.RejectRole)
            retval = msgBox.exec_()
            if retval == 2 and type(event) != bool:
                event.ignore()
        else:
            self.close()
   
    def deleteEvent(self, event):
        global bagFile
        global videoCSV
        if bagFile:
            player.videobox[frameCounter].removeAllBox()
            player.videoWidget.repaint()
    
    def copyPrevious(self):
        if frameCounter > 0:
            for i in xrange(frameCounter - 1, 0, -1):
                box = player.videobox[i]
                if box.box_id:
                    for j in xrange(i + 1, frameCounter + 1):
                        player.videobox[j].copy(box) 
                    break
            player.videoWidget.repaint()
            gantChart.axes.clear()
            gantChart.drawChart(player.videobox, framerate)
            gantChart.draw()
    
    def shortcuts(self):
        self.shortcuts = videoShortcuts.videoShortCuts()
        self.shortcuts.show()
        
    def edit_labels(self):
        self.editLabels = editLabels.editLabels()
        self.editLabels.show()     
        
                

if __name__ == '__main__':
    os.system('cls' if os.name == 'nt' else 'clear')
    app = QApplication(sys.argv)
    
    player = VideoPlayer()
    mainWindow = MainWindow(player)
    mainWindow.show()

    app.exec_()
    try:
        csvFileName = audioGlobals.bagFile.replace(".bag","_audio.csv")
        if audioGlobals.saveAudio == False:
            saveAudioSegments.save(csvFileName, audioGlobals.wavFileName)
    except:
        pass
