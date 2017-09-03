# This folder deals with the robotic arm

Contents:
* Arm.py
* FABRIK demo.py
* inverseKinematicUtilities.py

### General Inforamtion about each file

### Arm.py
This file contains the displayArm class. This class combines what we have used in the inverseKinematicUtilities file to create one class that we can have inverse kinematics, forward kinematics, old locations and more. This file also contains a demo using the arm class. Here is what the demo looks like ![demo](https://raw.githubusercontent.com/Uvic-Robotics-Club/Utility/master/Images/arm%20demo.gif?raw=true "demo inside arm.py")

### FABRIK demo.py
This file is a demo that shows the arm moving around using the inverseKinematicUtilities file.

### inverseKinematicUtilities.py
This file contains helper functions for inverse kinematics. All of the functions in the class have been implemented inside the displayArm class. This is mostly for backwards compatability or if someone wants to use the functions for another purpose.

