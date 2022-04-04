# point_cloud_registration
This Python package uses a TurtleBot3 robot with an Intel Realsense D435i depth camera and tries to create point clouds of Gazebo environments.

## Installation
### Ubuntu 20.0.4, ROS2 Foxy distro
Install Gazebo
```
sudo apt install ros-foxy-gazebo-* ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup
source ~/.bashrc
```

Install TurtleBot3
```
sudo apt install ros-foxy-dynamixel-sdk ros-foxy-turtlebot3-msgs ros-foxy-turtlebot3 ros-foxy-turtlebot3-gazebo
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
source ~/.bashrc 
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
colcon build --symlink-install
source ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc 
```

Check if everything is working
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run turtlebot3_teleop teleop_keyboard 
ros2 launch turtlebot3_bringup rviz2.launch.py
```

Install [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense) and [Intel RealSense ROS wrapper](https://github.com/IntelRealSense/realsense-ros)
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git -b foxy
cd ~/ros2_ws/
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build && . install/setup.bash
```

Check if everything is working
```
ros2 launch realsense2_description view_model.launch.py model:=test_d435i_camera.urdf.xacro
```

Install [Intel RealSense Gazebo plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
```
cd colcon_ws/src/
git clone -b foxy-devel https://github.com/pal-robotics/realsense_gazebo_plugin
cd ~/colcon_ws/
colcon build && . install/setup.bash
```

Install Octomap server
```
sudo apt install ros-foxy-octomap ros-foxy-octomap-msgs octovis
source ~/.bashrc
cd ~/ros2_ws/src/
git clone https://github.com/iKrishneel/octomap_server2.git
cd ~/ros2_ws/
colcon build && . install/setup.bash
```

## Working with this repo
```
cd ~/colcon_ws/src/
git clone https://github.com/Marina-Banov/point_cloud_registration.git
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/colcon_ws/src/point_cloud_registration/sdf/models' >> ~/.bashrc
source ~/.bashrc
cd ~/colcon_ws/
sudo apt install ros-foxy-sensor-msgs-py pcl-tools
pip install open3d transformations
colcon build && . install/setup.bash
```

The launch file loads a Gazebo world, positions the TurtleBot inside of it, and opens the Rviz visualization tool.
```
ros2 launch point_cloud_registration point_cloud_registration.launch.py
```
The default world is env3. Other available worlds are:
- empty
- env8
- env9
- house

which you can load like this:
```
ros2 launch point_cloud_registration point_cloud_registration.launch.py world:=env8
```

Open another terminal and run the following line to start the node which will assemble the point cloud from data received from `/depth/color/points` topic over time.
```
ros2 run point_cloud_registration get_pcd
```

Open another terminal and run the following line to start [TurtleBot3 teleop node](https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py) which allows you to control the Turtlebot.
```
ros2 run turtlebot3_teleop teleop_keyboard
```
Note that the `get_pcd` node will only process **ONE** frame after the `S` or `SPACE` key is pressed. Move the robot around and watch the octomap of the environment being generated. Press `CTRL+C` inside the `get_pcd` terminal to save the point cloud.

