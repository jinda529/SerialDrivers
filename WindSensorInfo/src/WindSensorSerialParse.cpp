#include <ros/ros.h>
#include <ros/timer.h>
#include <stdio.h>
#include <math.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <serial_port/data.h>
#include <common/WindSensorData.h>

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
common::WindSensorData wind_sensor_data;

//读取数据指令
unsigned char request_buf[8] = {0x02, 0x03, 0x00, 0x2A, 0x00, 0x05, 0xA4, 0x32};
float ser_data = 0; 
size_t ser_num = 0;
uint32_t ser_count = 0;
unsigned char ser_buffer[1024]; //读取的结果

/******** timeout protect defination *************/
uint32_t timeout_period = 40;
uint32_t loadMotor_timeout_count = 0;

bool data_parse_successful(size_t buffer_length, unsigned char* buffer) {
    if (buffer_length != 15 || (int) buffer[2] != 10) {
        return 0;
    }
    wind_sensor_data.timestamp = ros::Time::now().toSec();
    wind_sensor_data.temperature = ((double) ((int) buffer[3] * 256 + (int) buffer[4])) / 10.0;
    wind_sensor_data.humidity = ((double) ((int) buffer[5] * 256 + (int) buffer[6])) / 10.0;
    wind_sensor_data.pressure = ((double) ((int) buffer[7] * 256 + (int) buffer[8])) / 10.0;
    wind_sensor_data.wind_speed = ((double) ((int) buffer[9] * 256 + (int) buffer[10])) / 10.0;
    wind_sensor_data.wind_direction = ((double) ((int) buffer[11] * 256 + (int) buffer[12])) / 10.0;
    return 1;
}

int main (int argc, char** argv)
{

    ros::init(argc, argv, "WindSensorInfo");
    ros::NodeHandle nh;
    ros::Publisher WindSensorInfo_pub = nh.advertise<common::WindSensorData>("WindSensorInfo", 1000);
    
    try {
    //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");                
    } else {
        return -1;
    }

    //指定循环的频率200Hz
    ros::Rate loop_rate(100);
    while(ros::ok()) {
        
        ser_count++;        
        if (ser.isOpen())
        {
            std::cout << "ser_count is " << ser_count << std::endl;
            if (ser_count == 1)
            {
                ser.write(request_buf,8);
            } 
            if (ser_count == 9 && ser.available())
            {
                ser_num = ser.read(ser_buffer, ser.available());
                if (data_parse_successful(ser_num, ser_buffer)) {
                    std::cout << wind_sensor_data << std::endl;
                }

                WindSensorInfo_pub.publish(wind_sensor_data);
                std::cout << wind_sensor_data << std::endl;
                std::cout << "===========================" << std::endl;
            }  
            if (ser_count == 10)
            {
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

