# Usage - Quick start

1) Run the ARDrone 2.0 simulation on gazebo:
```
$roslaunch drone_construct main.launch
```
Gazebo will start and the drone will be wpawned in a football field map. 
In case that the drone is not spawned execute:
```
$roslaunch drone_construct put_drone_in_world.launch
``` 
The drone can take-off or land by publishing an empty message on /ardrone/takeoff and /ardrone/land topic respectively:
```
Taking-off:
$rostopic pub -1 /ardrone/takeoff std_msgs/Empty
Landing:
$rostopic pub -1 /ardrone/land std_msgs/Empty
```

2) Drone navigation:
In this repository there are two recomended ways to control the drone manually, using a 
ps3 joystick or using the the terminal virtual keyboard.
```
For PS3 Joystick:
$roslaunch ardrone_joystick teleop.launch
For virtual keyboard:
$rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
``` 

3) In case of a real drone the ARDrone 2.0 should be connected first via wifi with the computer and then the  driver should be executed.
```
$roslaunch real_drone real_drone.launch
```

4) Drone images:
Two are the recommended ways to show the images received from the drone.
```
Using the rqt_image_viewtool
$rosrun rqt_image_view rqt_image_view
Using the image_view tool
$rosrun image_view image_view image:=/ardrone/down_camera/image_raw
```
In order to change between the frontal and the downward camera output of the real drone, send first the following request to toggle the main camera.
```
$rosservice call /ardrone/togglecam 
```

5) Use a usb camera in ROS:
```
$roslaunch usb_cam usb_cam-test.launch
```
The node publishes the camera output on the /usb_cam/image_raw/ topic

6) Camera calibration:
For calibrating any camera for use in ROS first you need a chessboard calibration image with known dimentions [download one here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf). Next, follow the described proccess [link](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).
**NOTE:** This proccess creates a .yaml file with all the calculated coeficients for the specific camera. Sometimes, to use this file in other packages (i.e. ORB_SLAM2) this file should be converted in a different format: 
- [OpenCV](https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html). 
- [ROS camera Calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- [Conversion from ROS to OpenCV format](https://github.com/raulmur/ORB_SLAM2/issues/332)
> Your first "data" non zero non one elements are: fx, cx, fy, cy.Your second "data" is: k1, k2, p1, p2, k3 (irrelevant).The rest is also irrelevant.




# Troubleshooting (Dependances)
NOTE: the following are required:

- sudo apt-get install libspnav-dev (https://github.com/ros-drivers/joystick_drivers/issues/79)
- sudo apt-get install libsdl2-dev  (https://askubuntu.com/questions/626280/fatal-error-sdl-sdl-h-no-such-file-or-directory)
- sudo apt-get install libsdl1.2-dev
- sudo apt-get install python3-empy (https://answers.ros.org/question/257757/importerror-no-module-named-em-error/)
- sudo apt-get install ros-noetic-teleop-twist-keyboard
