#include <ros/ros.h>
#include <ros/timer.h>
#include <stdio.h>
#include <math.h>
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <serial_port/data.h>
#include <common/ProtractorData.h>

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>

#include <inttypes.h>
#include <Eigen/Eigen>

using namespace std;
using namespace ros;
using namespace Eigen;

serial::Serial ser; //声明串口对象
//serial_port::data Serial_Data;
common::ProtractorData protractor_data;

//读取数据指令
float ser_data = 0;
size_t ser_num = 0;
uint32_t ser_count = 0;
unsigned char ser_buffer[1024]; //读取的结果

/******** timeout protect defination *************/
uint32_t timeout_period = 40;

bool data_parse_successful(size_t buffer_length, unsigned char *buffer){
    double axis_1_angle_degree, axis_2_angle_degree;

    if (buffer_length < 32){
        std::cout << buffer_length << std::endl;
        return 0;
    }

    axis_1_angle_degree = double((buffer[7]-44) * 
                                 ((buffer[8]-48) * 10.0 + 
                                  (buffer[9]-48) + 
                                  (buffer[11]-48) * 0.1 + 
                                  (buffer[12]-48) * 0.01 + 
                                  (buffer[13]-48) * 0.001));
    axis_2_angle_degree = double((buffer[23]-44) * 
                                 ((buffer[24]-48) * 10.0 + 
                                  (buffer[25]-48) + 
                                  (buffer[27]-48) * 0.1 + 
                                  (buffer[28]-48) * 0.01 + 
                                  (buffer[29]-48) * 0.001));
    protractor_data.timestamp = ros::Time::now().toSec();
    protractor_data.axis_1_angle_degree = axis_1_angle_degree;
    protractor_data.axis_2_angle_degree = axis_2_angle_degree;
    return 1;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "ProtractorInfo");
    ros::NodeHandle nh;
    ros::Publisher ProtractorInfo_pub = nh.advertise<common::ProtractorData>("ProtractorInfo", 1000);

    try {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else {
        return -1;
    }

    //指定循环的频率10Hz
    ros::Rate loop_rate(100);
    while (ros::ok()){
        ser_count++;
        std::cout << ser_count << std::endl;
        if (ser.isOpen()) {
            // std::cout << "ser_count is " << ser_count << std::endl;
            if (ser_count == 9 && ser.available()) {
                ser_num = ser.read(ser_buffer, ser.available());
                std::cout << ser_num << std::endl;
                // int index = 0;
                // while (index < ser_num) {
                //     std::cout << std::hex << static_cast<int>(ser_buffer[index]) << std::endl;
                //     index ++;
                // }
                if (data_parse_successful(ser_num, ser_buffer)) {
                    std::cout << protractor_data << std::endl;
                    std::cout << ros::Time::now() << std::endl;
                }
                std::cout << "===========================" << std::endl;
            }
            if (ser_count == 10) {
                ser_count = 0;
            }
        }
        else {
            std::cout << "ser.Open() is false" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
