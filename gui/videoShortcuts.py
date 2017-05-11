#!/usr/bin/env python
# -*- coding: utf-8 -*-

from gui import table
from PyQt5.QtWidgets import QWidget

class videoShortCuts(QWidget, table.Ui_Form):
    
    def __init__(self, parent=None):
        super(videoShortCuts, self).__init__(parent)
        self.setupUi(self)
