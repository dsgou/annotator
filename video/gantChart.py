#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import itertools
from video.videoGlobals import videoGlobals
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QSizePolicy


class videoGantChart(FigureCanvas):
    
    def __init__(self, parent=None, width=15, height=1, dpi=100):
        gantChart = Figure(figsize=(width, height), dpi=dpi)
        self.axes = gantChart.add_subplot(111)
        self.drawChart([], None)
        
        FigureCanvas.__init__(self, gantChart)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


#Class for the gantChart
class gantShow(videoGantChart):
    
    #Plot the chart
    def drawChart(self, videobox, framerate):
        temp_action = []
        timeWithId = [[] for count in xrange(len(videobox))] 
        tickY = []
        tickX = []
        self.axes.hlines(0,0,0)
        
        for i in xrange(len(videobox)):
            frame_index = videobox[i]
            time_append = timeWithId[i].append
            for j in xrange(len(frame_index.box_id)):
                boxIdx = frame_index.box_id[j]
                if boxIdx != -1:
                    for allactions in frame_index.annotation[j]:
                        if isinstance(allactions, list):
                            for action in allactions:
                                time_append([boxIdx, action])
                        else:
                            time_append([boxIdx, allactions])

        boxAtYaxes = sorted(t for i in timeWithId for t in i)
        boxAtYaxes = list(k for k,_ in itertools.groupby(boxAtYaxes))
        
        ticky_append = tickY.append
        for key in xrange(len(boxAtYaxes)):
            ticky_append(key)
        
        if len(boxAtYaxes) > 0:
            for tup in boxAtYaxes:
                idx, action = tup[0], tup[1]
                index = 0
                start_time = 0
                end_time   = 1
                hor_lines = self.axes.hlines
                get_color = self.getColor
                time_calc = self.timeCalc
                while start_time < end_time:
                    start_time, end_time, index = time_calc(timeWithId, idx, index, action)
                    color = get_color(action)
                    hor_lines(boxAtYaxes.index([idx, action]), start_time, end_time,linewidth=10,color=color)
                    

        for tick in self.axes.yaxis.get_major_ticks():
            tick.label.set_fontsize(9)

        self.axes.set_xticklabels([])
        self.axes.set_yticks(tickY)
        self.axes.set_ylim([-1,len(boxAtYaxes)])
        self.axes.set_yticklabels([str(index[0]) + "::" + str(index[1]).ljust(5) for index in boxAtYaxes])
        self.axes.grid(True)

    #Calculates the end time for each annotation to plot
    def timeCalc(self, box_list, idx, index, action):
        temp_id = idx
        start_time = 0
        end_time = 0
        
        for i in xrange(index, len(box_list)-1): 
            flag = False
            index = i
            box_tuple = box_list[i]
            for entry in box_tuple:
                if temp_id == entry[0] and action == entry[1]:
                    flag = True
                    if start_time == 0:
                        start_time = i
                    break
            if not flag and start_time != 0:
                break
        if start_time != 0:
            end_time = index - 1
        return start_time, end_time, index

    #Calculates the color for the gantChart and bound Boxes
    def getColor(self, label):
        color = '#0000FF'
        if label in videoGlobals.classLabels:
            color = videoGlobals.annotationColors[videoGlobals.classLabels.index(label) % len(videoGlobals.classLabels)]
        elif label in videoGlobals.highLabels:
            color = videoGlobals.eventColors[videoGlobals.highLabels.index(label) % len(videoGlobals.highLabels)]
        return color
