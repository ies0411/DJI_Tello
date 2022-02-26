# VM_Tello

# 1. overview
<hr> Tello Drone algorithm for ausnomous flight. Basically, It needs a tello driver ROS, ORB SLAM Package.

This package would work based on ROS melodic ver framework.

## launch
- ``` $ roslaunch vm_tello vm.launch```

## Build from source

- ```$ cd <WORK SPACE>```
- ```$ git clone -b <version> https://github.com/viewmagine/VM_Tello.git```
- ```$ cd ..```
- ```$ catkin_make```
- ```$ source devel/setup.bash```

# 2. Dependancy

## tello driver
- ROS melodic version
- ```$ git clone https://github.com/surfii3z/tello_driver.git```

## Image distortion
- Beforce this package is to be installed, tello camera calibration must be preceded.
- ```$ git clone https://github.com/surfii3z/image_undistort ```

## Aruco Marker
- When drone land, it uses arucomarker
- ```& git clone -b melodic-devel https://github.com/pal-robotics/aruco_ros.git```

**todo param function**
