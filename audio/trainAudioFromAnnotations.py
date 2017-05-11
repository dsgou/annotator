from pyAudioAnalysis import audioTrainTest as aT

if __name__ =='__main__':
    #train
    aT.featureAndTrain(['Silence/','Speech/', 'Music/', 'Activity/'], 1.0, 1.0, aT.shortTermWindow, aT.shortTermStep, 'svm', 'svmModelTest', False)
