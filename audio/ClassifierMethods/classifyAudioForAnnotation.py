import os
import sys
import wave
import numpy as np
from pyAudioAnalysis import audioFeatureExtraction as aF
from pyAudioAnalysis import audioSegmentation as aS
from pyAudioAnalysis import audioBasicIO

#if __name__ =='__main__':
def Classify(wavFileName):
	#Segmatation and Classification
	#os.chdir('Home/Documents/python/audioGraph')
	[flagsInd, classesAll, acc] = aS.mtFileClassification(wavFileName, 'svmModelTest', 'svm', False)
	print flagsInd, classesAll
	#[segs, classes] = aS.flags2segs(flagsInd, acc)

	

