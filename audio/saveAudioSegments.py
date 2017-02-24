import os
import sys
import scipy.io.wavfile as wavfile
import csv
import operator
import wave
import numpy as np
from pyAudioAnalysis import audioFeatureExtraction as aF
from pyAudioAnalysis import audioTrainTest as aT
from pyAudioAnalysis import audioSegmentation as aS
from pyAudioAnalysis import audioVisualization as aV
from pyAudioAnalysis import audioBasicIO


def save(csv_, wav):
	#csv and wav file as argument
	print csv_, wav
	csvFileName = csv_
	wavFileName = wav

	Fs, x = audioBasicIO.readAudioFile(wavFileName)
	annotations = []
	silence = []
	folderName = None
	fileCounter = 0
	start, end = 0, 0

	#duration of wavFile
	spf = wave.open(wavFileName,'r')
	#Get wavFile duration
	frames = spf.getnframes()
	rate = spf.getframerate()
	duration = frames / float(rate)
	#duration = int((duration))


	csvFile = open(csvFileName, 'rb')
	# >> Empty csv file is 1 Byte -> check for empty
	if os.path.getsize(csvFileName) > 1:
		read = csv.reader(csvFile)
		#startTimeToPlay, endTimeToPlay -> str2float
		for row in read:
			row[0] = round(float(row[0])/1000,2)
			row[1] = round(float(row[1])/1000,2)
			#print row[0], row[1]
			if row[2][:8] == "Speech::":
				folderName = row[2][:6]
			else:
				folderName = row[2]

			annotations.append([row[0], row[1], folderName])
			#check if the directory exists and create it if necessary
			if not os.path.exists(folderName):
				os.makedirs(folderName)

		#sort annotations alphabetically based on class name
		annotations = sorted(annotations, key=operator.itemgetter(2), reverse=False)


		# >> Save audio segments in folders, based on annotation class
		for i,an in enumerate(annotations):
			#find file ID for existing files in directory to continue writing..
			directory = os.listdir(an[2])
			#check for empty directory
			if directory:
				index = directory[0].index('_')
				fileCounter = 0
				for i in range(len(directory)):
					if directory[i][index+1] > fileCounter:
						fileCounter = directory[i][index+1]
				fileCounter = int(fileCounter) + 1
			else:
				fileCounter = 0

			strOut = an[2] + "/{1:s}_{2:d}.wav".format(wavFileName.replace(".wav",""), an[2], fileCounter)
			fileCounter = fileCounter + 1
			#print strOut, int(Fs * an[0]), int(Fs * an[1])
			folderName = an[2]
			wavfile.write(strOut, Fs, x[int(Fs * an[0]):int(Fs * an[1])])

		# >> Find silence in audio file
		#sort annotations by start time
		annotations = sorted(annotations, key=operator.itemgetter(0), reverse=False)
		time = np.arange(0,duration,0.01)

		#Get silence before-between-after annotations
		for i in range(len(annotations)):
			tS = np.searchsorted(time, annotations[i][0])
			tE = np.searchsorted(time, annotations[i][1])
			end = round(time[tS],2)
			silence.append([start, end])
			start = round(time[tE],2)
		silence.append([start, duration])

		#remove overlapping 
		for i, s in enumerate(silence):
			if s[0]>s[1]:
				silence.remove(s)

		folderName = 'Silence'
		if not os.path.exists(folderName):
				os.makedirs(folderName)

		#find file ID for Silence
		directory = os.listdir(folderName)
		if directory:
			index = directory[0].index('_')
			fileCounter = 0
			for i in range(len(directory)):
				if directory[i][index+1] > fileCounter:
					fileCounter = directory[i][index+1]
			fileCounter = int(fileCounter) + 1
		else:
			fileCounter = 0

		#save silence segment
		for i, s in enumerate(silence):
			strOut = folderName + "/Silence_{1:d}.wav".format(wavFileName.replace(".wav",""), fileCounter)
			fileCounter = fileCounter + 1
			wavfile.write(strOut, Fs, x[int(Fs * s[0]):int(Fs * s[1])])
		print 'Finish saving audio segments...'

		for root, dirs, files in os.walk("/mydir"):
		    for silenceFile in files:
				if sys.getsizeof(silenceFile) <= 44:
					os.remove(silenceFile)
		