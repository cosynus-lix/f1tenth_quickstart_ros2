This repository is for providing a quick start on using the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) to launch a F1tenth virtual competition using ROS2. For a ROS1 version quick start, please refer to [f1tenth_quickstart version ROS1](https://github.com/cosynus-lix/f1tenth_quickstart).

# Quick demonstration by using example controller
## Preparation:
The simulator can be built in 3 different ways:
- natively installed as a ROS package
- built as a Docker image (**without** NVIDIA GPU)
- built as a Docker image (**with** NVIDIA GPU)

For purpose of simplicity, we will assume the 3rd option is used. For the other 2 options, please refer to the README file of [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros). Similarly, controllers can be built either natively or within a Docker container. In this tutorial, we set up controllers in a Docker container.

Build the Docker image for the simulator:
```
$ git clone git@github.com:f1tenth/f1tenth_gym_ros.git
$ cd f1tenth_gym_ros
$ docker build -t f1tenth_gym_ros -f Dockerfile .
``` 

## Single vehicle mode:
### Step 1 - Launch the simulator
Launch the Docker image for the simulator (`f1tenth_gym_ros`). You might need to take a few seconds to wait for RVIZ to fully function.
``` 
$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
### Step 2 - Launch the controller
In another terminal, clone this repository and build a Docker image (`f1tenth_quickstart_ros2`) for controllers, then launch it:
```
$ git clone git@github.com:cosynus-lix/f1tenth_quickstart_ros2
$ cd f1tenth_quickstart_ros2
$ docker build -t f1tenth_quickstart_ros2 -f Dockerfile .
$ docker run -it --rm --volume $(pwd)/src/:/ctrl_ws/src/ f1tenth_quickstart_ros2
```
Note that the folder `src` is mounted in the container using `--volume $(pwd)/src/:/ctrl_ws/src/`. Any changes to the `/ctrl_ws/src/` folder within the Docker environment will be automatically saved.

In the same Docker image, run example controller `wall_following`:
```
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 run wall_following sing_vehicle_mode
```
You can also use keyboard as the controller to move the ego vehicle:
```
$ sudo apt-get install ros-foxy-teleop-twist-keyboard
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Step 3 - Pose reset
To reset vehicle's pose, use `2D Pose Estimate` in RVIZ. 

## Head-to-head mode, ego + opp:
### Step 1 - Modify the configuration file, recompile simulator package and launch it:
```
$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
$ vim src/f1tenth_gym_ros/config/sim.yaml (change 'num_agent:' to 2)
$ colcon build
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Step 2 - Launch the controller
In another terminal, launch the docker image and run the example controller:
```
$ docker run -it --rm --volume $(pwd)/src/:/ctrl_ws/src/ f1tenth_quickstart_ros2
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 run wall_following head_to_head_mode
```
If the opponent vehicle does not appear correctly in RVIZ, try to add it manually: click `Add` at the right bottom of RVIZ window -> select `RobotModel` -> change `description topic` to `/opp_robot_description`.

### Step 3 - Add/remove obstacles and pose reset
To reset vehicles' pose, use `2D Pose Estimate` in RVIZ for the ego vehicle and `2D Goal Pose` for the opponent vehicle. 

# Change the track map of simulator
### Step 1
Copy map files (e.g. `berlin.png` and `berlin.yaml` in the [map library](https://github.com/f1tenth/f1tenth_simulator/tree/master/maps)) to the folder `maps/` in `f1tenth_gym_ros`.

### Step 2
Modify the map configuration, there are 2 options:

option 1 (temporary modification) - Modify the configuration file inside Docker image:
```
$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
$ vim src/f1tenth_gym_ros/config/sim.yaml (change 'map_path:' to the new map name)
$ colcon build
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
option 2 (permanent modification) - Modify the configuration before compiling Docker image:
```
$ (in host PC terminal) cd f1tenth_gym_ros
$ vim config/sim.yaml (change 'map_path:' to the new map name)
$ docker build -t f1tenth_gym_ros -f Dockerfile .

$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

You can also DIY a map (design a new one or add obstacles on an old one) by drawing pixels on map image (png/pgm/... files) under any picture editor! Just remember: white for free space and black for obstacles. Moreover, in the `.yaml` file, choose a small value for `resolution` and set the third coordinate of `origin` to a null value (e.g. `0`).

It should just work perfectly! For more details on changing maps, we refer to the description [here](https://github.com/f1tenth/f1tenth_gym_ros#changing-maps).


# Write the controller code by yourself!
## What to change?
### Using Python
1. in `src/wall_following/`: modify the file `single_vehicle.py` / `head_to_head.py`, or write an independent file with a similar structure.
2. the main point is to get environment information from the topic `/scan`(Message type - [`sensor_msgs/LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html), [more details](http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData)) and(or) `/odom`(Message type - [`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html), [more details](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)), calculate the controll command and send to the topic `/drive` (Message type - [`ackermann_msgs/AckermannDrive`](http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html), [more details](http://wiki.ros.org/Ackermann%20Group)).
3. file `setup.py`: in case that you write a node in a new python file e.g. `new_node.py`, you need to indicate the entry for launching this node:
```
    entry_points={
        'console_scripts': [
            'sing_vehicle_mode = wall_following.single_vehicle:main',
            'head_to_head_mode = wall_following.head_to_head:main',
            'new_node_mode' = wall_following.new_node:main
        ],
    }
```
4. file `package.xml`: in case that you use new dependancies (such as `ackermann_msgs` etc), add them in this file.
### Using C++
you can also write the controller in C++, refer to [ROS2 Tutorial](https://docs.ros.org/en/foxy/Tutorials.html).

## What is missing in the example code?
1. in the example code, we only use the information of Lidar via the topic `/scan` but not `/odom`. You can use `/odom` to obtain the ego agent's odometry for your own algorithm!
2. for head-to-head mode, we suppose that the ego and opp vehicle use the same controller and put their controller inside a same file. Of course you can use different controllers for each! In practice, it is better to write their controllers in different files.