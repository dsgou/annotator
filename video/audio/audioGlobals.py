class audioGlobals:
	#a few globals...

	#Argument files
	wavFileName = None
	rosbagName = None
	bagFile = None
	saveAudio = False

	#Signal
	spf = 0
	duration = 0 
	signal = 0

	#audio signal
	spf = 0
	duration = 0
	signal = 0

	#variables for start-end time, get mouse clicks and play
	xStart = 0
	xEnd = 0
	startTimeToPlay = 0
	endTimeToPlay = 0

	#Clicks variables
	counterClick = 1

	#Waveform and Gantt Chart figures for widgets and plot
	fig = None
	chartFig = None

	#Player flags for QtMultimedia player options
	playerStarted = False
	durationFlag = 0
	xPos = 0
	yPos = -580

	#Annotation colors
	colorName = None
	#list of green shades
	GreenShades = ['#007300', '#00e500', '#8CDD81', '#006600', '#659D32', '#007f00','#005900','#00cc00','#004c00', '#004000', '#009900', '#003300', '#00b200', '#002600', '#9AFF9A', '#337147']
	shadesAndSpeaker = []
	greenIndex = 0

	#Annotations important variables
	classLabels = ['Music', 'Activity', 'Laugh', 'Cough', 'Moan', 'Steps', 'TV']
	annotationFlag = False
	annotations = []
	timeArrayToPlot = []
	isBold = False
	isDeleted = False
	checkYaxis = False
	xTicks = []
	text_ = None
	overlap = False
	text1, text2 = None,None
	selected = False
	xCheck = None
	isAbove = False

	#Message Box Widget for info display
	msgDialog = None