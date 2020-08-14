
# RoboDKiiwaInterface

A driver used to control KUKA iiwa robots, the 7R800 and the 14R820, from inside your RoboDK simulation.


--------------------------------------

# Contents

The package contains two main folders:

1- RoboDKscript: contains the python files that shall be added to your RoboDK simulation.

2- SunriseController: contains the java files which shall be synchronized to the robot controller using your Sunrsie.Workbench.

--------------------------------------

# Video tutorials

Videos on the driver will be available [in this YouTube channel](https://www.youtube.com/channel/UCWYKX4o8Vfs6TkFrtaPo6Ag).

--------------------------------------

# Package Setup

To setup the RoboDKiiwaInterface follow the steps:

### Installing the driver into the robot controller:
1- You will need an external PC with the Sunrise.Workbench installed on it.

2- Establish a peer to peer connection between the external PC and the robot, [described in videos 1 and 2 of the play list](https://www.youtube.com/playlist?list=PLz558OYgHuZd-Gc2-OryITKEXefAmrvae);

3- Make sure that you have the Direct.Servo installed in your kuka iiwa as shown [in this video](https://youtu.be/v1LvxZ9GbRw);

4- Synchronise the [RoboDKiiwaInterface.java Application](https://github.com/Modi1987/RoboDKiiwaInterface/tree/master/SunriseController) into the robot controller;

### Installing the GUI into RoboDK on PC side:
1- Make sure that you have Python3.7 installed and recognized by your RoboDK.

2- Prepare your RoboDK simulation with the KUKA iiwa robot.

3- Add the python script [RoboDKiiwaInterface_verXX.py script](https://github.com/Modi1987/RoboDKiiwaInterface/tree/master/RoboDKscript) into your RoboDK simulation.

4- After adding it, double click the python script RoboDKiiwaInterface_verXX from inside RoboDK, and you shall see a graphical interface. 


--------------------------------------

# Robot Control

After setting up the driver, you can perfrom the following steps to control the robot from your RoboDK simulation:

### On the robot side:
1- Run the application "RoboDKiiwaInterface" from the smartPad of the robot.

2- You have 60 seconds to connect from the Python script (in your RoboDK) before the server is shut down automatically.

### From RoboDK side on your PC
From inside the python GUI (of the script RoboDKiiwaInterface_verXX) click on the button connect (after filling in the GUI with the correct values - Robot IP and Tool's parameters).

--------------------------------------

Copyright: Mohammad Safeea, first commit 14-08-2020

Note: This is still an expermintal package
