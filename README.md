This is a temporary repo of wind sensor data publisher.

1. clone this repo under a catkin workspace, such as ~/catkin_ws

2. make sure drive docker is updated to the latest, and your internet is good

3. (roscore in another terminal, or connect to ros master)

4. cd $(THE_CATKIN_WORKSPACE)

5. bash $(CATKIN_WORKSPACE)/src/SerialDrivers/launch_serial_docker.sh

6. bash $(CATKIN_WORKSPACE)/src/SerialDrivers/run_miran_sensor_node.sh
