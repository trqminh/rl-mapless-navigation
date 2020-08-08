# rl-mapless-navigation

## 1. Basic setup
- Install ROS Melodic (for Ubuntu 18.04)
- Install Gazebo9
- Install [gazebo_ros package](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
## 2. Client setups (Gazebo GUI, turtlebot3)
### 2.1. Installation and turtlebot3 setups: 
I followed this [tutorial](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/), in brief:
- Install turtlebot3 and turtlebot3_msgs:
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
```
- Export the turtlebot model in your .bashrc, e.g:
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source .bashrc
```
- Install turtlebot3_simulations:
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```
### 2.2. turtlebot3 urdf
- Modify the .urdf.xacro file in catkin_ws/src/turtlebot3/turtlebot3_description/urdf for robot plugins (sensors, ...), my urdf is available [here](./misc/turbot_urdf)
### 2.3. Worlds and launch files
- Create worlds by gazebo and save as .world files
- Add .world files in git to /catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
- Add .launch files in git to /catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
- My world and launch files are avalable [here](./misc/worlds) and [here](./misc/launch), in turn   

Using my setups by running:
```
sh misc/transfer.sh
```
## 3. Server setups
### 3.1. Python for ROS
I followed this [link](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674), for cv_bridge and other issues, and with some modifications for virtualenv:   
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
- Type following commands:
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
### 3.2. Python for RL
- Tensorflow gpu 1.15
```
pip install tensorflow-gpu==1.15
```
- Torch and torchvision
```
pip install torch==0.4.1 torchvision==0.2.1
```
### 3.3. Connect two machines (if the server and the client are in separate computers)
- Follow the first answer of this [link](https://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/)
- Test the connection by rostopic echo to subscribe to some public topic in client
## 4. Training and evaluation
### 4.1. In client's terminal
```
source ~/catkin_ws/devel/setup.bash
```
- Launching turtlebot to the training world, e.g:
```
roslaunch turtlebot3_gazebo train_env2.launch
```
- or Launching turtlebot to the testing world, e.g:
```
roslaunch turtlebot3_gazebo test_env2.launch
```
### 4.2. In server's terminal
```
source ~/catkin_build_ws/install/setup.bash --extend
```
then:
```
python src/baseline.py
```
--train = 1 for training, 0 for testing   
--visual_obs 1 for using camera, 0 for using laser scan   
--env_id is training environment id   
--test_env_id is testing environment id   
- For examples:   

Training in the training environment id 2 with camera
```
python src/baseline.py --visual_obs 1 --training 1 --env_id 2
```
Testing above training models in the testing environment id 2
```
python src/baseline.py --visual_obs 1 --training 0 --env_id 2 --test_env_id 2
```
## 5. Utilities
### Mapping the testing world
- I followed this [link](https://newscrewdriver.com/2018/08/11/running-turtlebot3-mapping-demonstration-with-a-twist/#:~:text=Note%3A%20If%20this%20node%20failed,%2Dkinetic%2Dslam%2Dgmapping%20.) for mapping a gazebo world. Launch a test world and run this:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
- then run this and start mapping
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
- Eventually, save the map for visualizing path in need
```
rosrun map_server map_saver -f ~/map
```
### Plot training results
```
python utils/draw.py
```
### Make video from mono vision frames
```
python utils/make_video.py
```
### Public paths for visualization
```
python utils/path.py
```
