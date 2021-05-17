# INOF(InterOffice) Mobile Robot 
A robot which moves inside of an office to deliver various things like documents,files or mails.
Tools & Languages:
 - ROS 1 (Robot Operating System, ver. Noetic Ninjemys) and Gazebo
 - C++
 - Python
 

## Applications
  Run following command to control robot via movements-commands. Please do necessary [network-configuration](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) 
```sh
roslaunch inof_robot launch_cmd_control.launch
```


Run following command to control robot via PS4 Dualshock controller
```sh
roslaunch inof_robot launch_joystick_control.launch
```


Note: You may find other files, unused or for different applications and tests. Ignore them, or you may enjoy exploring them!