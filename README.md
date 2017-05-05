# multimedia-annotator

Annotate video and audio streams either in common formats(mp4, avi, mkv, wav, mp3) or rosbag files. Rosbag format is used by ROS(http://www.ros.org/) to group multiple streams in a single file. 

You can easily annotate video files per frame or in groups by forming rectangle areas. In the example below we have a screenshot of the program in use for and mp4 file. You can see the video player and a gant chart of the annotations we have made along with the corresponding waveform of the audio stream and its automatically extracted annotations(we will talk about that later).
![alt text](https://drive.google.com/file/d/0B7MZy1Not-DyamFRek8tUkROR2c/view?usp=sharing)

## Installation
 * Dependencies
 
   * rosbag
   * python-pyqt5
   * python-pyqt5.qtmultimedia
   * libqt5multimedia5
   * libqt5glib
   * python-matplotlib
   * python-numpy
   * libav-tools
   * pyAudioAnalysis (found on github https://github.com/tyiannak/pyAudioAnalysis)

## Usage
   * Running the annotator
 ```
     python annotator.py
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
 
