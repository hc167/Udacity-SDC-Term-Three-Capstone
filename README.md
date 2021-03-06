## TheRacingSDC ##

- Hiu Chan - Team Lead (hiu_chan@hotmail.com)
- Yuda Wang (yuda@berkeley.edu)
- Ran Yan (896521722@qq.com)
- Zhening (Sirius) Zhang (zznzhang@ucdavis.edu)
- Praveen Kumar Marothu (praveen.marothu@gmail.com)

#### Youtube Video Demo: https://www.youtube.com/watch?v=h8KuF9HO1Jw

#### Big Thanks to: #### 
- https://github.com/leggedrobotics/darknet_ros , darknet_ros, which is used in the project.
- And https://pjreddie.com/darknet/yolo/ , Joseph Redmon, the original author of darknet & YOLO.


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)


### Prerequisite for Udacity VM with ROS pre-installed
Apply the following commands (one at a time)
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

### Prerequisite for Udacity Workspace
Apply the following commands (one at a time)
```bash
apt update
apt install ros-kinetic-dbw-mkz-msgs
```

## ROS Architecture

ROS node architecture with the publish and subscribe topics. 

![alt text](./Car_system.png)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/hc167/Udacity-SDC-Term-Three-Capstone.git Capstone0
```

2. Install python dependencies
```bash
cd Capstone0
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
