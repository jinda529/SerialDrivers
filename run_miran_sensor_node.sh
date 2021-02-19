sudo apt-get install ros-melodic-serial;
sudo apt-get install libserial-dev;
sudo chmod 777 /dev/ttyUSB0;
cd $( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )/../../;
catkin_make;
source ./devel/setup.bash;
rosrun MiranSensorInfo MiranSensorInfo;
