# multimedia-annotator

Annotate video either in common formats or rosbags.
Annotate video(RGB and depth), audio and laser measurements stored in rosbags.

## Installation
 * Dependencies
 
   * ros-kinetic-rospy
   * ros-kinetic-cv-bridge
   * ros-kinetic-vision-opencv
   * ros-kinetic-image-geometry
   * ros-kinetic-python-qt-binding
   * python-pyqt5
   * python-pyqt5.qtmultimedia
   * libqt5multimedia5
   * libqt5glib
   * python-matplotlib
   * python-numpy
   * libav-tools
   * pyAudioAnalysis (found on github)

##Usage
   * Running the annotator
 ```
     python rosbagAnnotator.py
```
 * Video annotation actions   
   * Creating an annotation area
    * Click and drag the mouse pointer over the area
   * Annotating/editing an area
    * Right click on the area
   * Add/Remove Labels
    * Edit > Edit Labels  
   * Import previous work
    * File > Import video csv (after selecting rosbag)
    
  * Output Files are written at the rosbag file directory  
      
## Usage example: video
 * Run the annotator
 * Open rosbag file
 * Select the topic corresponding to the video stream from the rosbag
 * Annotate video frames by clicking and dragging over the area you want to annotate
 * Right click in that area and select the annotation label
 * Continue to annotate until the end of the video
 * Save by going at File > Save Video Csv
 
