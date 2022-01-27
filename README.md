## For compile

drive `--`volume /dev:/dev -i docker.plusai.co:5050/plusai/drive-p1.1:20220117  
cd  catin_ws/src  
git clone git@github.com:jinda529/SerialDrivers.git  
cd  ..  
source /opt/ros/melodic/setup.bash  
sudo apt-get update
sudo apt-get `--`assume-yes install ros-melodic-serial  
sudo apt-get `--`assume-yes install libserial-dev  
sudo chmod 777 /dev/ttyUSB0  
sudo chmod 777 /dev/ttyUSB1  
sudo chmod 777 /dev/ttyUSB2  
sudo chmod 777 /dev/ttyUSB3  
catkin_make -DCATKIN_WHITELIST_PACKAGES=`"`common`"`  
catkin_make -DCATKIN_WHITELIST_PACKAGES=`"`MiranSensorInfo`"`  
 catkin_make -DCATKIN_WHITELIST_PACKAGES=`"`ProtractorInfo`"`   
catkin_make -DCATKIN_WHITELIST_PACKAGES=`"`SteeringWheelSensorInfo`"`  
catkin_make -DCATKIN_WHITELIST_PACKAGES=`"`WindSensorInfo`"`  
##  For run
cd  catin_ws/  
source ./devel/setup.bash  
rosrun MiranSensorInfo MiranSensorInfo
