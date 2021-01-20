### Stuart Jessup
## Robot Programming Assignment
January 2021

## Specialism
This assignment focuses on the navigation specialism.


## Overview

The code consists of a mapping and a traversing object, which are run sequentially.

The mapping object creates a map of the field.  The robot moves randomly around the field using reactive collision avoidance.  

The  traversing object drives the robot up and down the field in rows. The camera image is processed using a blob detector to locate weeds. The location of a weed is queued and when the sprayer is calculated to have moved over the weed, the sprayer is activated.  

A look-ahead is performed before the start of each traverse. The occupancy map is examined and the distance to be travelled is truncated if an obstacle is found in the robot’s path.

## Operation
The code uses launch files from a fork of the CMP9767M repository and is zipped with this fork.


### 1) Launch Gazebo:
Basic spraying and traversing is best demonstrated with 
roslaunch uol_cmp9767m_base thorvald-sim.launch fake_locatisation:=True map_server:=True obstacles:=False

### 2) Launch localisation, mapping and move_base:
roslaunch uol_cmp9767m_tutorial mapping_fake_localisation.launch

### 3) Run program:
rosrun my_examples move_avoid_zap.py

(Program can be run with obstacles set true in gazebo, but operation is unreliable.  I would be interested to get feedback on what is happening in this case, how to set up the simulation appropriately, and how to improve performance.)

### Zipped Files
move_avoid_zap.py  
CMP9767M_master repo fork, note that models have been removed to reduce size
readme.md (this file)

### GitHub repositories
Code can also (unofficially) be found at 
https://github.com/Bluboy456/Robot-Programming.git
https://github.com/Bluboy456/robot_prog_tutorial_fork.git
