# map_scan
Using ray-cast to generate virtual LaserScan data in given map, only work for 2D robot.

# Instructions
map_scan is a ros package which implements `ray-cast` to generate virtual `LaserScan` data in given map. It's very appriciated that the [amcl](https://github.com/ros-planning/navigation.git) had implemented the beam ray-cast wonderfully. And in this package, it will use the beam ray-cast provided by amcl and just modify it a little.

# Usage
**Note:** To use this package, you could pull down the code to your computer. However, I should tell you that the package only has been tested in Ubuntu 16.04 and Ubuntu 18.04 corresponding to `ROS` distribution of `kinetic` and `melodic`.

## step 1
New a folder as your workspace. Below is the example.
```Shell
> mkdir -p ~/raycaster/src
> cd ~/raycaster/src
```

## step 2
Pull down the code to your workspace.
```Shell
> git clone 
```

## Step 3
Compile the code.
```Shell
> cd ~/raycaster
> catkin_make
```
## Step 4
If you have sussecced to compile the code. Congratulations. And you can run the test code to feel how it works.
```Shell
> source devel/setup.bash
> roslaunch map_scan test.launch
```
And you can see something like below.  
![screenshot](https://github.com/hefvcjm/map_scan/raw/master/screenshot/virtual_scan_test.png)
# API

## Services Called
static_map ([nav_msgs/GetMap](http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html))  
map_scan calls this service to retrieve the map that is used for virtual-laser-based localization; startup blocks on getting the map from this service.

## Services
map_scan([map_scan/GetMapScan](https://github.com/hefvcjm/map_scan/blob/master/srv/GetMapScan.srv))  
Service to get the virtual-laser-scan in the map given the pose and the info of the virtual-laser.

