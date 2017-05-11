#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QPushButton, QWidget, QLineEdit, QFormLayout, QVBoxLayout, QMessageBox, QHBoxLayout

#Class for box id change
class changeBoxId(QWidget):

    def __init__(self, videobox, index, frameCounter, framerate, gantChart):

        QWidget.__init__(self)
        self.box_Idx = None
        self.videobox = videobox
        self.index = index
        self.frameCounter = frameCounter
        self.framerate = framerate
        self.gantChart = gantChart
        
        self.cancel = QPushButton("Cancel", self)
        self.cancel.clicked.connect(self.closeTextBox)
        self.Ok = QPushButton("Ok", self)
        self.Ok.clicked.connect(self.pressedOk)
        
        self.boxId = QLineEdit(self)
        self.boxId.textChanged.connect(self.boxChanged)
        self.boxId.setPlaceholderText('Box Id:')
        self.boxId.setMinimumWidth(80)
        self.boxId.setEnabled(True)
        self.boxId.move(90, 15)

        flo = QFormLayout()
        flo.addRow(self.boxId)
        
        boxLayout = QHBoxLayout()
        boxLayout.addWidget(self.cancel)
        boxLayout.addWidget(self.Ok)
        
        verLayout = QVBoxLayout()
        verLayout.addLayout(flo)
        verLayout.addLayout(boxLayout)
        
        self.setLayout(verLayout)
        self.setWindowTitle('Set Box id')
        self.show()
       
    def boxChanged(self,text):
        self.box_Idx = text
        
    def closeTextBox(self,text):
        self.close()

    def pressedOk(self):
        try:
            self.box_Idx = int(self.box_Idx)
            previous_id  = self.videobox[self.frameCounter].box_id[self.index]
            #Check id
            if self.box_Idx in self.videobox[self.frameCounter].box_id:
                #Box Id already given
                msgBox = QMessageBox()
                msgBox.setText("Box Id already given")
                msgBox.setIcon(msgBox.Warning)
                msgBox.setWindowTitle("Error")
                msgBox.exec_()
            else:
                while self.frameCounter < len(self.videobox):
                    if(self.index < len(self.videobox[self.frameCounter].box_id)):
                        if previous_id == self.videobox[self.frameCounter].box_id[self.index]:
                            self.videobox[self.frameCounter].box_id[self.index] = self.box_Idx
                    self.frameCounter += 1
                self.gantChart.axes.clear()
                self.gantChart.drawChart(self.videobox, self.framerate)
                self.gantChart.draw()
                self.Ok.clicked.disconnect()
                self.close()
        except:
            msgBox = QMessageBox()
            msgBox.setText("Wrong type, integer expected")
            msgBox.setIcon(msgBox.Warning)
            msgBox.setWindowTitle("Error")
            msgBox.exec_()
            self.close()
