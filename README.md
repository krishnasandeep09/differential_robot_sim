# differential_robot_sim
--------------------------------A Differential Robot in ROS and Gazebo--------------------------------------

1. About the project

A differential robot with two drivable wheels at the front and two casters at the rear is used.
The robot has a 2D Lidar at the front, an RGBD camera and a screen attached to a pole.

The package contains the following,

    (1) Robot description (urdf) and world
    (2) Meshes and materials (for Hokuyo Lidar and Kinect camera)
    (3) Config and Launch files (explained in detail in later sections)
    (4) src: the scripts

The robot can be moved using 3 methods, namely

    (1) Teleop keyboard
    (2) Explore map automatically
    (3) Navigate from Point A to B, while avoiding obstacles
    
Additional info:

    (1) During autonomous movement, the robot can be enabled or disabled using a ROS service
    (2) The robot can perform SLAM (for example using the gmapping package)
    (3) The movement of the robot, map being built (if gmapping is installed) and the camera feed can be visualized in Rviz.

2. How to build

Clone the repository into your workspace's src folder and then run
  
    (1) cd /your_ws
    (2) catkin_make

Install teleop-twist-keyboard and slam-gmapping packages to use keyboard operation and LiDAR SLAM respectively. The following commands can be used to install the packages. Replace distro with your ros distro name

    sudo apt-get install ros-<distro>-teleop-twist-keyboard
    sudo apt-get install ros-<distro>-slam-gmapping

3. How to run

As mentioned earlier the robot can be in 3 ways,

a) Manual Teleop control

    (1) roslaunch my_robot gazebo.launch #spawns the robot
    (2) roslaunch my_robot slam.launch #starts gmapping
    (3) roslaunch my_robot teleoperator.launch #starts teleop keyboard
    
b) Map Exploration

    roslaunch my_robot explorer.launch #spawns robot, starts gmapping and starts exploring the world
    
c) A to B Navigation

    roslaunch my_robot navigator.launch # spawns robot, starts gmapping and starts moving from A to B

d) Enable/Disable the robot using "/set_robot_status" service

Note: Source the devel space using "source devel/setup.bash", before launching the files

4. More details about the logic

a) Map Exploration
    The robot moves straight until it reaches an obstacle and turns left/right as per feasibility
    In order to explore more of the world and not stop at circumnavigating the walls of the world, a random turn is introduced at random intervals

b) A to B Navigation
    The robot moves homing towards the goal direction greedily
    While avoiding obstacles it turns first to the direction it has turned when it first homed towards the goal. 
