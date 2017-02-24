#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
from gui import picklist
from gui import addLabels

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
from video.videoGlobals import videoGlobals


#Class for class addition
class addLabel(QWidget, addLabels.Ui_Dialog):

    def __init__(self, editLabel, isHighLevel, parent=None):
        super(addLabel, self).__init__(parent)
        self.setupUi(self)
        
        self.label = None
        self.editLabel = editLabel
        self.isHighLevel = isHighLevel
        
        self.lineEdit.textChanged.connect(self.boxChanged)
        self.lineEdit.setPlaceholderText('Class Label:')
        
       
    def boxChanged(self, text):
        self.label = text
        
    def reject(self):
        self.close()

    def accept(self):
        self.label = self.label
        if self.label in videoGlobals.classLabels or self.label in videoGlobals.highLabels:
            msgBox = QMessageBox()
            msgBox.setText("Class label already exists")
            msgBox.setIcon(msgBox.Warning)
            msgBox.setWindowTitle("Error")
            msgBox.exec_()
        else:
           videoGlobals.classLabels.append(self.label)
           self.close()
           color = QColorDialog.getColor()
           json_data = None
           with open("labels.json", 'r+') as json_file:
               json_data = json.load(json_file)
               if self.isHighLevel:
                    json_data['highlevellabels'].append(self.label)
                    json_data['eventColors'].append(color.name())
               else:
                    json_data['basiclabels'].append(self.label)
                    json_data['annotationColors'].append(color.name())
           with open("labels.json", 'w+') as json_file:    
               json.dump(json_data, json_file)
        
        videoGlobals.classLabels, videoGlobals.highLabels, videoGlobals.annotationColors, videoGlobals.eventColors = self.parseJson()
        self.editLabel.refreshLists()
        
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

    
class editLabels(QWidget, picklist.Ui_Form):
    
    def __init__(self, parent=None):
        super(editLabels, self).__init__(parent)
        self.setupUi(self)
        self.refreshLists()
        
        self.pushButton.clicked.connect(self.moveRight)
        self.pushButton_2.clicked.connect(self.moveLeft)
        self.pushButton_3.clicked.connect(self.done)
        self.add_basic.clicked.connect(self.addBasicLabel)
        self.add_high.clicked.connect(self.addHighLabel)
        
        self.pushButton_6.clicked.connect(self.moveRightHigh)
        self.pushButton_4.clicked.connect(self.moveLeftHigh)
        self.pushButton_5.clicked.connect(self.done)
    
    def refreshLists(self):
        self.listWidget_3.clear()
        self.listWidget_5.clear()
        for label in videoGlobals.classLabels:
            self.listWidget_3.addItem(label)
        for label in videoGlobals.highLabels:
            self.listWidget_5.addItem(label)
    
    def addBasicLabel(self):
        self.addEvent = addLabel(self, False)
        self.addEvent.show() 
        
        
    def addHighLabel(self):
        self.addEvent = addLabel(self, True)
        self.addEvent.show() 
    
        
    def moveRight(self):
        selection = self.listWidget_3.takeItem(self.listWidget_3.currentRow())
        self.listWidget_4.addItem(selection)
        
    def moveLeft(self):
        selection = self.listWidget_4.takeItem(self.listWidget_4.currentRow())
        self.listWidget_3.addItem(selection)
        
    def moveRightHigh(self):
        selection = self.listWidget_5.takeItem(self.listWidget_5.currentRow())
        self.listWidget_6.addItem(selection)
        
    def moveLeftHigh(self):
        selection = self.listWidget_6.takeItem(self.listWidget_6.currentRow())
        self.listWidget_5.addItem(selection)
        
    def done(self):
        json_data = {"basiclabels":[],"annotationColors":[], "highlevellabels":[], "eventColors":[]}
        
        colors = []
        basicLabel = []
        for i in range(self.listWidget_3.count()):
            item = self.listWidget_3.item(i)
            basicLabel.append(item.text())
            colors.append(videoGlobals.annotationColors[i])
        json_data['basiclabels'] = basicLabel
        json_data['annotationColors'] = colors
        
        colors = []
        highLabel = []
        for i in range(self.listWidget_5.count()):
            item = self.listWidget_5.item(i)
            highLabel.append(item.text())
            colors.append(videoGlobals.eventColors[i])
        json_data['highlevellabels']= highLabel
        json_data['eventColors'] = colors
        
        with open("labels.json", 'w+') as json_file:    
               json.dump(json_data, json_file)
        videoGlobals.classLabels, videoGlobals.highLabels, videoGlobals.annotationColors, videoGlobals.eventColors = self.parseJson()
        self.close()
        
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
        
