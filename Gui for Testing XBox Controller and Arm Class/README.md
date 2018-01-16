# This folder is for a demo of Xbox control, HTTP Transfer, and FK and IK Arm calculations

Contents:
* 360_controller.png
* Controls.ui
* Main Program.py
* Main Window Resource.qrc
* Main Window.ui
* vectors.py
* allNone.xbox
* default.xbox
* latestImage.png
* requirements.txt
* settings.py
* xbox_controller.py
* xbox_controller_worker.py

### General Inforamtion

### Main Program.py
This file is the main starting point of this project and allows you to "trace" where the program goes.
Essentially what this program does is start 3 threads. Something to get all of the inputs from the xbox controller, Something to watch the network and make sure we are listening for incomming connections, and then to start the GUI.

![demo](https://raw.githubusercontent.com/Uvic-Robotics-Club/Utilities/wheel_arduino/Images/Arm%20Demo%20with%20Xbox%20controller.gif?raw=true)

### UI files
These are files that QT designer creates. Essentially they are the GUI's definitions.

### vectors.py
This is a helper class that allows more concice and clear vectors to be handled instead of just using an array that has 3 elements.
