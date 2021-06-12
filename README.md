# INOF(INterOFfice) Mobile Robot 
A friendly machine which delivers office essentials, from files to coffee! 

<br/>

**INOF's Hardware**
 - Raspberry Pi zero W
 - BO Motors and L298N motor controller IC
 - HC-04 Ultrasonic Sensor
 - Raspberry Pi 5MP camera
 
<br/>

**Tools & Languages:**
 - ROS 1 (Robot Operating System, distro. Noetic Ninjemys ver. 1.15.11) and Gazebo (ver. 11.5.1)
 - C++
 - Python
 - OpenCV

<br/>

**Brief Implemetation Breakdown:**
<br/>
&rarr; Path Planning <br/>
&nbsp;&nbsp;&nbsp;&rarr; Motion Planning (Moving from one point to another) <br/>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&rarr; Movement of robot (Forward, Backward, Rotate) i.e. controlling motors <br/>  

<br/><br/>
## Applications
**Application-A:** <br/>
Run the following command to control INOF via movement-commands. Please do necessary [network-configurations] to your system (laptop/PC).
```sh
roslaunch inof_robot launch_cmd_control.launch
```
Give the following command from network-configured system.<br/>
Command : `rostopic pub -1 /control_cmd std_msgs/String -- "F:1"`
<br /> 
- `-1` &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&nbsp; : Publish only once
- `/control_cmd`     &emsp; &nbsp;: Topic name to publish command on
- `std_msgs/String` : Message type for the topic
- `"F:1"`&emsp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp; : Message. 'F'&rarr; Forward, 'B'&rarr; Backward, 'R'&rarr; Rotate.  
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'1'&rarr; motion for 1 second, '30'&rarr; Rotation of 30 degrees

Find the demo video [here](https://youtu.be/pgnShXFkE3s)



<br/>
<br/>

**Application-A-Extended:** <br/>
In this application, [pyPS4Controller library] has been used. Follow the guide by [pyPS4Controller library] to pair PS4 remote controller with INOF's controller for once.

Run the following command to control robot via PS4 remote controller. 
```sh
roslaunch inof_robot launch_ps4controller.launch
```
Find the demo video [here](https://youtu.be/NiBOTVwJ394)



<br/>
<br/>

**Application-B:** <br/>
In this application, the INOF is programmed to follow the pre-registered object (A yellow ball). 

Run the following command to control robot via PS4 remote controller. 
```sh
roslaunch inof_robot launch_object_follower.launch
```
Find the demo video [here](https://youtu.be/pgnShXFkE3s)



_Note: You may find other files which are unused, under-development or for different applications and tests. Ignore them, or you may enjoy exploring them!_


[network-configurations]: <http://wiki.ros.org/ROS/Tutorials/MultipleMachines>
[pyPS4Controller library]: <https://pypi.org/project/pyPS4Controller>


