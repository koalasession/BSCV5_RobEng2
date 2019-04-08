# BSCV5_RobEng2
Ros Indigo Project using a turtlebot2 to map and navigate using LiDAR

### Installation

In your workspace src folder clone the repo with this command.

```sh
git clone https://github.com/koalasession/BSCV5_RobEng2.git
```

then in your catkin_ws folder.

```sh
catkin_make
```

and update ros profile

### Scripts

- **move.py** : Publish Twist messages.
- **square.py** : Publish Odometry goals.
- **move_base_square.py** : Uses move_base.

#### Run
bring up turtlebot
```sh
roslaunch turtlebot_bringup minimal.launch --screen
```

on workstation
```sh
rosrun mbot_motion <script_name>.py
```

for **move_base_square.py** you need to make sure a move_base server is running, either by providing a map through map_server or running gmapping or using AMCL.
