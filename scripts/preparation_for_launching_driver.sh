sudo apt-get update
sudo apt-get --assume-yes install ros-melodic-serial;
sudo apt-get --assume-yes install libserial-dev;
sudo chmod 777 /dev/ttyUSB0;
sudo chmod 777 /dev/ttyUSB1;
sudo chmod 777 /dev/ttyUSB2;
sudo chmod 777 /dev/ttyUSB3;
cd $( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )/../../;
catkin_make;
source ./devel/setup.bash;
