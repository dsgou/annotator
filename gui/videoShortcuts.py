#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
from gui import table

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
from video.videoGlobals import videoGlobals

class videoShortCuts(QWidget, table.Ui_Form):
    
    def __init__(self, parent=None):
        super(videoShortCuts, self).__init__(parent)
        self.setupUi(self)
