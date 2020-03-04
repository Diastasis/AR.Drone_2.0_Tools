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
rostopic pub -1 /ardrone/land std_msgs/Empty
```

2) Drone navigation:
In this repository there are two recomended ways to control the drone manually, using a 
ps3 joystick or using the the terminal virtual keyboard.
```
For PS3 Joystick:
$ roslaunch ardrone_joystick teleop.launch
For virtual keyboard:
$rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
``` 

3) In case of a real drone the ARDrone 2.0 should be connected first via wifi with the computer and then the  driver should be executed.
```
roslaunch real_drone real_drone.launch
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
rosservice call /ardrone/togglecam 
```





# Troubleshooting
NOTE: the following are required:

- sudo apt-get install libspnav-dev (https://github.com/ros-drivers/joystick_drivers/issues/79)
- sudo apt-get install libsdl2-dev  (https://askubuntu.com/questions/626280/fatal-error-sdl-sdl-h-no-such-file-or-directory)
