# multimedia-annotator

Annotate video and audio streams either in common formats(mp4, avi, mkv, wav, mp3) or rosbag files. Rosbag format is used by ROS(http://www.ros.org/) to group multiple streams in a single file.

You can easily annotate video files per frame or in groups by forming rectangle areas. In the example below we have annotated an mp4 file. You can see the video player and a gant chart of the annotations we have made along with the corresponding waveform of the audio stream and its automatically extracted annotations(see the wiki).
![example](https://cloud.githubusercontent.com/assets/4816678/25743986/0f111eca-31a0-11e7-8b82-80a1994c48d1.png)


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

 
