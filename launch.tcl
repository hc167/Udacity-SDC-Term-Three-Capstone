cd ros
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.sh
roslaunch launch/styx.launch
