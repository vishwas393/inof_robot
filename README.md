# INOF(INterOFfice) Mobile Robot 
A robot which moves inside of an office to deliver various things like documents,files or mails.

**INOF's Hardware**
 - Controller: Raspberry Pi zero W
 - BO Motors and L298N motor controller IC
 - HC-04 Ultrasonic Sensor
 - Raspberry Pi 5MP camera
 

**Tools & Languages:**
 - ROS 1 (Robot Operating System, ver. Noetic Ninjemys) and Gazebo
 - C++
 - Python


**Brief Implemetation Breakdown:**
&rarr; Path Planning
&nbsp;&nbsp;&nbsp;&rarr; Motion Planning (Moving from one point to another)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&rarr; Movement of robot (Forward, Backward, Rotate) i.e. controlling motors  


## Applications
**Application-A:**
Run following command to control INOF via movements-commands. Please do necessary [network-configuration] to your system (laptop/PC).
```sh
roslaunch inof_robot launch_cmd_control.launch
```
Give the following command from network-configured system
Command : `rostopic pub -1 /control_cmd std_msgs/String -- "F:1"`
&nbsp;&nbsp;&nbsp; - `-1` &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp; : Publish only once
&nbsp;&nbsp;&nbsp; - `/control_cmd` &ensp;&ensp;&ensp; : Topic name to publish command on
&nbsp;&nbsp;&nbsp; - `std_msgs/String` : Message type for the topic
&nbsp;&nbsp;&nbsp; - `"F:1"`&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp; : Message. 'F'&rarr; Forward, 'B'&rarr; Backward, 'R'&rarr; Rotate.  
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'1'&rarr; motion for 1 second, '30'&rarr; Rotation of 30 degrees

Find the video here


**Application-A-Extended:**
Here [pyPS4Controller library] has been used. Follow the guide by pyPS4Controller to pair PS4 controller with INOF's controller for once.

Run following command to control robot via PS4 Dualshock controller. Also, press home button on PS4 conrtoller to get connected to INOF's controller's bluetooth module. 
```sh
roslaunch inof_robot launch_ps4controller.launch
```
Find the video here

Note: You may find other files, unused, under-development or for different applications and tests. Ignore them, or you may enjoy exploring them!


[network-configuration]: <http://wiki.ros.org/ROS/Tutorials/MultipleMachines>
[pyPS4Controller library]: <https://pypi.org/project/pyPS4Controller>

