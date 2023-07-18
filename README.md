# dvrk2robosuite

## Prerequisite
Ubuntu 20.04  
[Anaconda](https://docs.anaconda.com/free/anaconda/install/linux/) (to install robosuite in virtual environment)  
[Robosuite](https://robosuite.ai/docs/installation.html)  
[ROS desktop-full](http://wiki.ros.org/noetic/Installation/Ubuntu)  
[catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) (for catkin build)  

## Building the packages b4 using (don't use catkin_make)
```bash
conda activate robosuite  
catkin clean
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3  
source ./devel/setup.bash  
```

## step 1: launch dvrk master and GC on master side

## step 2: launch simulator/mtm_interface on simulator side
```
roslaunch dvrk2robosuite teleop.launch
```
or
'''
roslaunch dvrk2robosuite teleop_record.launch
'''
for data record

## Some Useful CMD
```bash
roscd dvrk2robosuite/   
conda activate robosuite  
source devel/setup.bash  
rosrun dvrk2robosuite simulator.py  
```