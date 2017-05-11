from __future__ import unicode_literals

import matplotlib

matplotlib.use('Qt5Agg')
from PyQt5 import QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


from audioGlobals import audioGlobals

annotationColors = (['Speech', 'green'],['Music','red'], ['Activity', 'magenta'],['Laugh', 'yellow'], ['Cough', '#4B0082'], ['Moan', '#800000'], ['Steps', '#FFA500'], ['TV', '#6F4E37'])

class ChartWindow(FigureCanvas):
    def __init__(self, parent=None, width=15, height=1, dpi=100):
        figChart = Figure(figsize=(width, height), dpi=dpi)
        self.axes = figChart.add_subplot(111)

        self.drawChart()

        FigureCanvas.__init__(self, figChart)
        #self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def drawChart(self):
        pass

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  GANTTCHART PLOT FUNCTION                                                               #
#  called from application window                                                         #
#  draw annotations in Ganttchart                                                         #
#  not clickable window                                                                   #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

class Chart(ChartWindow):

    labels = []
    classesToPlot = []
    tickY=[]
    tickX=[]
    timeArrayToPlot = []
    c = None
    length = 0
    fileExist = False

    # >> PLOT GANTTCHART
    def drawChart(self):
        global annotationColors
        self.axes.clear()
        self.classesToPlot = []
        self.labels = []
        self.tickY = []
        self.tickX = []

        #find annotations classes and save them for plot
        #----------------------
        for index1 in range(len(audioGlobals.annotations)):
            self.labels.append(audioGlobals.annotations[index1][2])

        #remove duplicates
        self.classesToPlot = list(set(self.labels))

        #sort classesToPlot list alphabetically
        self.classesToPlot = sorted(self.classesToPlot)
        self.length = len(self.classesToPlot)


        #Create y and x ticks
        for index in range(self.length):
            self.tickY.append(index + 1)

        for index in range(len(audioGlobals.timeArrayToPlot)):
            self.tickX.append(index + 1)


        self.axes.hlines(0,0,0)
        #create object to plot
        for index in range(self.length):
            for anIndex in range(len(audioGlobals.annotations)):
                if self.classesToPlot[index] == audioGlobals.annotations[anIndex][2]:
                    for colorIndex in range(len(annotationColors)):
                        if annotationColors[colorIndex][0] == audioGlobals.annotations[anIndex][2]:
                            self.c = annotationColors[colorIndex][1]
                        elif audioGlobals.annotations[anIndex][2][:8] == 'Speech::':
                            if len(audioGlobals.shadesAndSpeaker) > 0:
                                for shadeIndex in range(len(audioGlobals.shadesAndSpeaker)):
                                    if audioGlobals.annotations[anIndex][2] == audioGlobals.shadesAndSpeaker[shadeIndex][0]:
                                        self.c = audioGlobals.shadesAndSpeaker[shadeIndex][1]
                            else:
                                self.c = audioGlobals.GreenShades[audioGlobals.greenIndex]
                                if audioGlobals.greenIndex >= len(audioGlobals.GreenShades):
                                    audioGlobals.greenIndex = 0
                                else:
                                    audioGlobals.greenIndex = audioGlobals.greenIndex + 1
                    self.axes.hlines(index + 1, (audioGlobals.annotations[anIndex][0]/1000), (audioGlobals.annotations[anIndex][1]/1000),linewidth=10, color=self.c)
            self.axes.hlines(index + 2,0,0)

        #Reverse Y axes once
        if audioGlobals.checkYaxis == False:
            self.axes.invert_yaxis()
            audioGlobals.checkYaxis = True

        #Small Font in Y Axes
        for tick in self.axes.yaxis.get_major_ticks():
            tick.label.set_fontsize(9) 

        self.axes.xaxis.tick_top()
        self.axes.set_xticks(audioGlobals.xTicks)
        self.axes.set_xticklabels([])
        self.axes.set_xlim([-1,audioGlobals.duration + 1])
        self.axes.set_yticks(self.tickY)
        self.axes.set_yticklabels(self.classesToPlot)
        self.axes.grid(True)
