# mapless-planner

## 1. Basic setup
- Install ROS Melodic (for Ubuntu 18.04)
- Install Gazebo9
- Install [gazebo_ros package](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
## 2. Install and setup turtlebot3:
Follow this [tutorial](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/), in brief:
- Install turtlebot3 and turtlebot3_msgs:
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
```
- Export the turtlebot model in your .bashrc, e.g:
```
export TURTLEBOT3_MODEL=burger
```
- Install turtlebot3_simulations:
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

## 3. More setup
### Adding plugin to your turtlebot3
- Modify the .urdf.xacro file in catkin_ws/src/turtlebot3/turtlebot3_description/urdf for robot plugins (sensors, ...), my urdf is available [here](./turbot_urdf)
### Create the world
- Create world by gazebo and save as .world file
- Add .world file in git to /catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
- Add .launch file in git to /catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
- My world and launch file are avalable [here](./worlds) and [here](./launch), in turn
### Setup python
I followed this [link](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674), for cv_bridge and other issues, and with some modifications for virtualenv, here is my setup:   
- Install pyyaml:
```
sudo apt-get install python3-pip python3-yaml
```
- In your active python3 venv:
```
pip3 install rospkg catkin_pkg
```
- Install some tools for the build process
```
sudo apt-get install python-catkin-tools python3-dev python3-numpy
```
- Type following:
```
mkdir ~/catkin_build_ws && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=~/myenv/bin/python3 -DPYTHON_INCLUDE_DIR=~/myenv/include/python3.6m -DPYTHON_LIBRARY=~/myenv/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so
catkin config --install
```
- clone and build:
```
mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd ~/catkin_build_ws
catkin build cv_bridge
source install/setup.bash --extend
```

- Test cv_bridge installation
```
from cv_bridge.boost.cv_bridge_boost import getCvType
```
- Remember sourcing the setup or add to the .bashrc directly
```
source ~/catkin_build_ws/install/setup.bash --extend
```
- Tensorflow gpu 1.15
```
pip install tensorflow-gpu==1.15
```
### Connect two machine (in case using server for training)
- Setup all above (no need gazebo) for server
- Follow the first answer of this [link](https://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/), (optional: add the export to .bashrc)
- Test connection by rostopic echo to subscribe to some public topic in client
## 4. Training and evaluation
### Training
- Launching turtlebot the training world
```
roslaunch turtlebot3_gazebo train_env2.launch
```
- then train
```
python src/baseline.py --train 1 --visual_obs 0 --env_id 2
```
### Mapping the testing world
I followed this [link][https://newscrewdriver.com/2018/08/11/running-turtlebot3-mapping-demonstration-with-a-twist/#:~:text=Note%3A%20If%20this%20node%20failed,%2Dkinetic%2Dslam%2Dgmapping%20.] for mapping a gazebo world. Launch a test world and run this:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
then run this and start mapping
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Eventually, save the map for visualizing path in need
```
rosrun map_server map_saver -f ~/map
```
### Testing
- Launching turtlebot the testing world
```
roslaunch turtlebot3_gazebo test_env1.launch
```
- Start rviz (optinal)
```
rviz
```
- Start drawing path (optional - run this on the client in case training on server)
```
python utils/path.py
```
- Run testing
```
python src/baseline.py --train 0 --visual_obs 0 --env_id 2 --test_env_id 1
```

