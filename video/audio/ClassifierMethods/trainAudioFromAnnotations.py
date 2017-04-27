import os
import sys
from pyAudioAnalysis import audioFeatureExtraction as aF
from pyAudioAnalysis import audioTrainTest as aT
from pyAudioAnalysis import audioBasicIO


#aT.featureAndTrain([""], 1.0, 1.0, aT.shortTermWindow, aT.shortTermStep, "svm", "svmMusicGenre3", True)
if __name__ =='__main__':

    #train
    aT.featureAndTrain(['Silence/','Speech/', 'Music/', 'Activity/'], 1.0, 1.0, aT.shortTermWindow, aT.shortTermStep, 'svm', 'svmModelTest', False)