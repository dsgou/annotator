import sys
import os
import random
import matplotlib
import matplotlib.pyplot as plt
import time
import math
import csv
import ast
#rom scipy import spatial

# Make sure that we are using QT5
matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QFile, QIODevice
from PyQt5.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QPushButton,
QSizePolicy, QVBoxLayout, QWidget,QLineEdit, QInputDialog, QMenu)

from numpy import arange
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

from enum import Enum
from laserGlobals import laserGlobals

progname = os.path.basename(sys.argv[0])

#Arxikopoihsh global timwn
#Arxikopoihsh metritwn
laserGlobals.cnt = 0
timer = None
#Arxikopoihsh grafikwn parastasewn
ax = None
fig = None
fw = None
scan_widget = None
ok = 'No'
#Arxikopoihsh annotation
objx = []
objy = []
annotating = False
firstclick = False
secondclick = False
thirdclick = False
colours = ['#FF69B4','#FFFF00','#CD853F','#000000','#8A2BE2','#00BFFF','#ADFF2F','#8B0000']
colour_index = 0
c1 = []
c2 = []
colorName = []
txt = None
annot = []
classes = None
selections = []
le = None
objs = None
reinitialize = False
rightClick = None
items = []
openline = False

class Window(FigureCanvas):

    def __init__(self, parent=None, width=10, height=3, dpi=100):

        global fw,fig,ax,bag_file,data

        fw = self

        fig = Figure(figsize=(width, height), dpi=dpi)

        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        fig.canvas.mpl_connect('button_press_event', onClick)

    def icon(self):
        pass

def onClick(event):

    global firstclick, c1, secondclick, c2,  thirdclick

    x = event.x
    y = event.y
    if event.button == Qt.LeftButton:
            if firstclick == False:
                if event.inaxes is not None:
                    c1 = [event.xdata,event.ydata]
                    firstclick = True
            elif secondclick == False:
                if event.inaxes is not None:
                    c2 = [event.xdata,event.ydata]
                    if (c2[0]<c1[0]):
                        temp_c = c2
                        c2 = c1
                        c1 = temp_c
                    secondclick = True
                    laserGlobals.ok = 'Rect'
                    laserGlobals.scan_widget.drawLaserScan()
            elif ((not thirdclick) and (secondclick)):
                #if event.button == Qt.RightButton:
                    #na emfanizei to menu me tis classes 
                    #na exei kai tin epilogi pros9ikis class
                if event.button == Qt.LeftButton:
                    laserGlobals.ok = 'Yes'
                    firstclick = False
                    secondclick = False
                    if ((laserGlobals.cnt>0) and (laserGlobals.cnt<len(laserGlobals.annot))):
                        laserGlobals.scan_widget.drawLaserScan()
    else:
        rightClick = event.x
        xPos = event.xdata
        yPos = event.ydata

        if (laserGlobals.ok == 'Rect'):

            menu = QMenu()
            deleteBox = menu.addAction('Delete Box')
            cancel = menu.addAction('Cancel')
            submenu = menu.addMenu('Choose Person')
            for i in range(len(items)):
                classes = submenu.addAction(items[i])
                #classes.triggered.connect(functools.partial(chooseClass,items[i]))
            changeId = submenu.addAction('Change Id')
            cancel.triggered.connect(cancelannot)
            changeId.triggered.connect(chId)
            deleteBox.triggered.connect(delBox)
            
        menu.exec_(laserGlobals.scan_widget.mapToGlobal(QtCore.QPoint(x, y)))
            


        #action = menu.exec_(mapToGlobal(event.pos()))

def contextMenuEvent(event):

    global ok, rightClick,items,classes,submenu

    rightClick = event.pos()

    if (laserGlobals.ok == 'Rect'):

        menu = QMenu(self)

        deleteBox = menu.addAction('Delete Box')
        deleteBox.triggered.connect(self.delBox)
        submenu = menu.addMenu('Choose Person')
        for i in range(len(items)):
            classes = submenu.addAction(items[i])
            classes.triggered.connect(functools.partial(self.chooseClass,items[i]))
        changeId = submenu.addAction('Change Id')
        changeId.triggered.connect(self.chId)
        cancel = menu.addAction('Cancel')
        cancel.triggered.connect(cancelannot)

        #action = menu.exec_(self.mapToGlobal(event.pos()))

def delBox():
    global firstclick,secondclick,cnt,annot,ok,scan_widget,le,Ok,openline
    firstclick = False
    secondclick = False
    if ((laserGlobals.cnt>=0) and (laserGlobals.cnt<len(laserGlobals.annot))):
        laserGlobals.ok = 'Yes'
        laserGlobals.scan_widget.drawLaserScan()
    if openline:
        le.close()
        Ok.close()
        openline = False

def chooseClass(txt_):
    global colour_index,annot,selections,txt,colours,ok,scan_widget,items
    colour_index = laserGlobals.annot[0].selections.index(txt_)%(len(colours))
    laserGlobals.ok = 'Yes'
    laserGlobals.scan_widget.training()


def chId():
    global le, rightClick,nw,Ok,openline
    openline = True
    le = QLineEdit()
    le.setDragEnabled(True)
    le.setPlaceholderText("Write ID:")
    le.move(700,100)
    le.show()
    Ok = QPushButton("Ok")
    Ok.move(700,150)
    #Ok.clicked.connect(showObject)
    Ok.show()

def cancelannot():
    global annot,cnt,samex,c1,c2,listofpointsx,listofpointsy,ok,scan_widget
    for i in range(len(laserGlobals.annot[laserGlobals.cnt].samex)):
        if ((laserGlobals.annot[laserGlobals.cnt].samex[i] >= c1[0]) and (laserGlobals.annot[laserGlobals.cnt].samex[i] <= c2[0]) and ((laserGlobals.annot[laserGlobals.cnt].samey[i] >= c2[1]) and (laserGlobals.annot[laserGlobals.cnt].samey[i] <= c1[1]))):
            if ((laserGlobals.annot[laserGlobals.cnt].listofpointsx[i] != []) and (laserGlobals.annot[laserGlobals.cnt].listofpointsy[i] != [])): #IndexError: list index out of range
                laserGlobals.annot[laserGlobals.cnt].listofpointsx[i] = []
                laserGlobals.annot[laserGlobals.cnt].listofpointsy[i] = []
    laserGlobals.ok = 'Yes'
    laserGlobals.scan_widget.drawLaserScan()

def showObject():
    global le,selections,classes,objs,sel_pos
    if le.text() not in selections:
        print classes
        classes.addAction(le.text())
        objs = le.text()
        selections.append(objs)

class LS(Window):

    def ptime(self):
        laserGlobals.timer = QtCore.QTimer(None)

        laserGlobals.timer.timeout.connect(self.icon)
        laserGlobals.timer.start(100)
    

    def icon(self):
        global ok
        #print laserGlobals.cnt
        if(laserGlobals.cnt<len(laserGlobals.annot)):
            laserGlobals.ok = 'Yes'
            laserGlobals.scan_widget.drawLaserScan()
            laserGlobals.cnt += 1
        if (laserGlobals.cnt == len(laserGlobals.annot)):
            laserGlobals.cnt=0
            laserGlobals.timer.stop()
            laserGlobals.ok = 'No'
            laserGlobals.scan_widget.drawLaserScan()

    def drawLaserScan(self):

        global ax,samex,samey,listofpointsx,listofpointsy,fw,ok,c1,c2,colorName,firstclick,secondclick,colourID,colorName

        if (laserGlobals.ok == 'Yes'):
            self.axes.clear()
            self.axes.axis('equal')
            self.axes.plot(laserGlobals.annot[laserGlobals.cnt].samex,laserGlobals.annot[laserGlobals.cnt].samey,'o')
            if not laserGlobals.annot[laserGlobals.cnt].listofpointsx == []:
                for j in range(len(laserGlobals.annot[laserGlobals.cnt].colourID)):
                    self.axes.plot(laserGlobals.annot[laserGlobals.cnt].listofpointsx[j],laserGlobals.annot[laserGlobals.cnt].listofpointsy[j],color=laserGlobals.annot[laserGlobals.cnt].colourID[j],marker='o')
            fw.draw()
        elif (laserGlobals.ok == 'Rect'):
            self.axes.axis('equal')
            if (laserGlobals.cnt>0) and (laserGlobals.cnt<len(laserGlobals.annot)):
                self.axes.plot([c1[0],c2[0]],[c1[1],c1[1]],'r')
                self.axes.plot([c2[0],c2[0]],[c1[1],c2[1]],'r')
                self.axes.plot([c2[0],c1[0]],[c2[1],c2[1]],'r')
                self.axes.plot([c1[0],c1[0]],[c2[1],c1[1]],'r')
                fw.draw()
        elif (laserGlobals.ok == 'No'):
            self.axes.clear()
            fw.draw()

    def training(self):

        global annot,samex,samey,c1,c2,colorName,colours,colour_index,colourID,listofpointsx,listofpointsy,ok,firstclick,secondclick, bag_file, data,selections

        for i in range(len(annot[laserGlobals.cnt].samex)):
            if ((annot[laserGlobals.cnt].samex[i] >= c1[0]) and (annot[laserGlobals.cnt].samex[i] <= c2[0]) and ((annot[laserGlobals.cnt].samey[i] >= c2[1]) and (annot[laserGlobals.cnt].samey[i] <= c1[1]))):
                colorName = colours[colour_index]
                annot[laserGlobals.cnt].colourID.append(colorName)
                annot[laserGlobals.cnt].listofpointsx.append(annot[laserGlobals.cnt].samex[i])
                annot[laserGlobals.cnt].listofpointsy.append(annot[laserGlobals.cnt].samey[i])
                #annot[laserGlobals.cnt].samex = [x for x in annot[laserGlobals.cnt].samex if x not in annot[laserGlobals.cnt].listofpointsx]
                #annot[laserGlobals.cnt].samey = [y for y in annot[laserGlobals.cnt].samey if y not in annot[laserGlobals.cnt].listofpointsy]
        ok = 'Yes'
        laserGlobals.scan_widget.drawLaserScan()
        colour_index+=1
        if (colour_index == (len(colours))):
           colour_index = 0 
        firstclick = False
        secondclick = False

        #SAVE to CSV
        filename = bag_file.replace(".bag","_laser.csv")
        with open(filename, 'w') as data:
            write = csv.writer(data)
            for row in annot:
                row_ = [row.samex, row.samey, row.listofpointsx, row.listofpointsy, row.annotID, row.colourID]
                write.writerow(row_)
            data.close()



class ApplicationWindow(QtWidgets.QMainWindow):

    def __init__(self):

        global  classes, le

        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

        self.main_widget = QtWidgets.QWidget(self)

        scan_widget = LS(self.main_widget)

        #Define buttons
        scanLayout = QHBoxLayout() 
        scanLayout.addWidget(scan_widget)

        playButton = QPushButton("Play")
        pauseButton = QPushButton("Pause")
        prevFrameButton = QPushButton("Previous")
        nextFrameButton = QPushButton("Next")
        stopButton = QPushButton("Stop")
        le = QLineEdit(self)
        le.setDragEnabled(True)
        addButton = QPushButton('Add', self)

        classes = QComboBox()
        classes.addItem('Classes')

        buttonLayout = QHBoxLayout()
        buttonLayout.addWidget(playButton)
        buttonLayout.addWidget(pauseButton)
        buttonLayout.addWidget(prevFrameButton)
        buttonLayout.addWidget(nextFrameButton)
        buttonLayout.addWidget(stopButton)
        buttonLayout.setAlignment(Qt.AlignTop)

        classLayout = QVBoxLayout()
        classLayout.addWidget(classes)
        classLayout.addWidget(le)
        classLayout.addWidget(addButton)
        classLayout.setAlignment(Qt.AlignTop)

        layout = QVBoxLayout(self.main_widget)
        layout.addLayout(scanLayout)
        layout.addLayout(buttonLayout)
        layout.addLayout(classLayout)

        #Define Connections
        playButton.clicked.connect(self.bplay)
        pauseButton.clicked.connect(self.bpause)
        prevFrameButton.clicked.connect(self.bprevious)
        nextFrameButton.clicked.connect(self.bnext)
        stopButton.clicked.connect(self.bstop)
        classes.activated[str].connect(self.chooseClass)
        addButton.clicked.connect(self.showObject)


        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)

    def bplay(self):
        global scan_widget
        scan_widget.ptime()

    def bpause(self):
        global timer
        timer.stop()

    def bprevious(self):
        global ok

        if (laserGlobals.cnt>0):
            laserGlobals.cnt = laserGlobals.cnt-1
            ok = 'Yes'
            laserGlobals.scan_widget.drawLaserScan()
        else:
            ok = 'No'
            laserGlobals.scan_widget.drawLaserScan()

    def bnext(self):
        global  annot, ok, scan_widget,colour_index
        colour_index = 0
        if (laserGlobals.cnt<len(annot)):
            laserGlobals.cnt = laserGlobals.cnt+1
            ok = 'Yes'
            laserGlobals.scan_widget.drawLaserScan()
        else:
            ok = 'No'
            laserGlobals.scan_widget.drawLaserScan()

    def bstop(self):
        global timer,ax,fw
        laserGlobals.cnt = 0
        timer.stop()
        ax.clear()
        fw.draw()


    def showObject(self):
        global le,selections,classes,objs,sel_pos
        if le.text() not in selections:
            classes.addItem(le.text())
            objs = le.text()
            selections.append(objs)

    def chooseClass(self,text):
        global scan_widget,txt,colour_index,colours,annot,selections
        txt = text
        if ((txt != 'Classes') and secondclick):
            annot[laserGlobals.cnt].annotID.append(txt)
            colour_index = selections.index(txt)%(len(colours))
            scan_widget.training()

class laserAnn:

    global c1,c2, objx,objy, s1,s2, txt

    def __init__(self, samex_=None, samey_=None, listofpointsx_=None,listofpointsy_=None, annotID_=None, colourID_=None):

        self.samex = []
        self.samey = []
        self.listofpointsx = []
        self.listofpointsy = []
        self.annotID = []
        self.colourID = []

        if samex_ == None:
            self.samex = []
        else:
            self.samex = samex_
        if samey_ == None:
            self.samey = []
        else:
            self.samey = samey_
        if listofpointsx_ == None:
            self.listofpointsx = []
        else:
            self.listofpointsx = listofpointsx_
        if listofpointsy_ == None:
            self.listofpointsy = []
        else:
            self.listofpointsy = listofpointsy_
        if annotID_ == None:
            self.annotID = []
        else:
            self.annotID = annotID_
        if colourID_ == None:
            self.colourID = []
        else:
            self.colourID = colourID_

def run(laserx,lasery,bagFile, filename):

    global timer,annot,s1,s2,bag_file,colorName

    timer = QtCore.QTimer(None) 
    bag_file = bagFile
    filename = filename.replace(".bag", "_laser.csv")
    if os.path.isfile(filename):
        with open(filename, 'rb') as data:
            if os.path.getsize(filename)>1:
                read = csv.reader(data)
                for row in read:

                    row[0] = row[0][1:-1]
                    row[1] = row[1][1:-1]
                    row[2] = row[2][1:-1]
                    row[3] = row[3][1:-1]
                    row[4] = row[4][1:-1]
                    row[5] = row[5][1:-1]
                    row[5] = row[5].replace("'","")
                    row[0] = row[0].split(", ")
                    row[1] = row[1].split(", ")
                    row[2] = row[2].split(", ")
                    row[3] = row[3].split(", ")
                    row[4] = row[4].split(", ")
                    row[5] = row[5].split(", ")

                    for i in range(0, len(row[0])):
                        if row[0][i] == "":
                            row[0] = []
                            break
                        else:
                            row[0][i] = float(row[0][i])
                    for i in range(0, len(row[1])):
                        if row[1][i] == "":
                            row[1] = []
                            break
                        else:
                            row[1][i] = float(row[1][i])
                    for i in range(0, len(row[2])):
                        if row[2][i] == "":
                            row[2] = []
                            break
                        else:
                            row[2][i] = float(row[2][i])
                    for i in range(0, len(row[3])):
                        if row[3][i] == "":
                            row[3] = []
                            break
                        else:
                            row[3][i] = float(row[3][i])
                    for i in range(0, len(row[4])):
                        if row[4][i] == "":
                            row[4] = []
                            break
                        else:
                            row[4][i] = str(row[4][i])
                    for i in range(0, len(row[5])):
                        if row[5][i] == "":
                            row[5] = []
                            break
                        else:
                            row[5][i] = str(row[5][i])

                    la = laserAnn(row[0], row[1], row[2], row[3], row[4], row[5])
                    laserGlobals.annot.append(la)
    else:
        for i in range(len(laserx)):
            s1 = laserx[i].tolist()
            s2 = lasery[i].tolist()
            la = laserAnn(samex_=s1,samey_=s2)
            laserGlobals.annot.append(la)

    #qApp = QtWidgets.QApplication(sys.argv)

    #s = ApplicationWindow()

    #s.show()

    #sys.exit(qApp.exec_())