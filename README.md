#Manipulation package with behavior trees

![](images/groot.png)

## Requierments and supported platforms
### Supported platforms/releases:

|Platform|ROS Release|
|-|------|
|[Ubuntu 18.04](https://releases.ubuntu.com/18.04/)|[ROS Melodic](http://wiki.ros.org/melodic/Installation)|

### Requirements: 

You should have the [swot package](https://developer.leapmotion.com/tracking-software-download), [Python 2](https://www.python.org/downloads/release/python-272) and [Python 3](https://www.python.org/downloads/) installed on your device. 

## Documentation and Enviornment setup
For guidance on installation, development, environmental setup, and troubleshooting, see our [documentation](). This documentation includes a description of the sensors and the robot arm which were used to develop and test the industrial-like interaction.

## Current features
- [Pick up and delivery of an object/piece while hand is being hold inside the sensor working area](/scripts/demonstrator_hold_hand)
- [Pick up an object once a signal is given to the robot](/scripts/Industrial_like_robot)
- [Delivery of an object once the a hand is detected again](/scripts/Industrial_like_robot)
- [Pick up and delivery of differnt objects/pieces placed in different locations]((/scripts/Industrial_like_robot))
- Determinations of the hands status

## Installation of this package
**1.** Go to the source folder of your catkin workspace.
```bash 
cd ~/catkin_ws/src
git clone https://github.com/Jasv06/human_robot_interaction.git
cd ~/catkin_ws
catkin_make
```
**2.** Source your current catkin workspace.
```bash 
source ~/catkin_ws/devel/setup.bash
```
**3.** Extract the LeapMotion folder and place it in your desired location outside your catkin working space.

**4.** Go to the [Leap_client_demonstrator.py](/scripts/demonstrator_hold_hand/Leap_client_demonstrator.py) file and change the path in line 8 based on the location where you placed the folder LeapMotion.

**5.** Go to the [Leap_client_industrial.py](/scripts/Industrial_like_robot/Leap_client_industrial.py) file and change the path in line 8 based on the location where you placed the folder LeapMotion.

## Running the scripts
If you didn't add `source $YOUR_WORKSPACE/devel/setup.bash` and `source /opt/ros/noetic/setup.bash` to your `.bashrc`, remember to source it when you open a new terminal. Also, in the following example catkin_ws is the name of our workspace, but this could change depending on the name you gave to your workspace.

### Example
#### Connecting the Leap motion sensor
1. Open a terminal and type:
```sh
sudo leapd
```
#### Use the Leap Motion sensor
2. Open a terminal, source it and type the following:
```sh
cd ~/catkin_ws/src/human_robot_interaction/scripts/Industrial_like_robot
python2 Leap_client_industrial.py
```
#### Publishing the data extracted from the Leap motion sensor into ROS
3. Open a new terminal, source it and type the following:
```sh
cd ~/catkin_ws
roslaunch human_robot_interaction leap_ros_industrial.launch
```
#### Connecting to the Robot
4. Ensure the Robot is connected correctly, open a new terminal, source it and type the following:
```sh
cd ~/catkin_ws
roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=rx150 
```
#### Controlling the robot
5. Finally, open one last terminal, source it and type the following:
```sh
cd ~/catkin_ws
rosrun human_robot_interaction robot_control_industrial_3.py
``` 
