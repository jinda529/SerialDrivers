#include <ros/ros.h>
#include <ros/timer.h>
#include <stdio.h>
#include <math.h>
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <serial_port/data.h>
#include <common/MiranSensorData.h>

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
common::MiranSensorData miran_sensor_data;

//读取数据指令
unsigned char request_buf[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
float ser_data = 0;
size_t ser_num = 0;
uint32_t ser_count = 0;
unsigned char ser_buffer[1024]; //读取的结果

/******** timeout protect defination *************/
uint32_t timeout_period = 40;

bool data_parse_successful(size_t buffer_length, unsigned char *buffer)
{
    if (buffer_length < 9)
    {
        return 0;
    }
    miran_sensor_data.timestamp = ros::Time::now().toSec();
    miran_sensor_data.length = ((double)buffer[3] * 256 * 256 * 256 + (double)buffer[4] * 256 * 256 + (double)buffer[5] * 256 + (double)buffer[6]) / 65536.0;
    return 1;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "MiranSensorInfo");
    ros::NodeHandle nh;
    ros::Publisher MiranSensorInfo_pub = nh.advertise<common::MiranSensorData>("MiranSensorInfo", 1000);

    // 检查是否有信号跳变（由于缓冲错误）
    double last_length = -1000.0;

    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    //指定循环的频率200Hz
    ros::Rate loop_rate(2000);
    while (ros::ok())
    {
        ser_count++;
        if (ser.isOpen())
        {
            // std::cout << "ser_count is " << ser_count << std::endl;
            if (ser_count == 1)
            {
                ser.write(request_buf, 8);
            }
            if (ser_count == 9 && ser.available())
            {
                ser_num = ser.read(ser_buffer, ser.available());
                // std::cout << ser_num << std::endl;
                int index = 0;
                while (index < ser_num) {
                    // std::cout << std::hex << static_cast<int>(ser_buffer[index]) << std::endl;
                    index ++;
                }
                if (data_parse_successful(ser_num, ser_buffer))
                {
                    std::cout << miran_sensor_data << std::endl;
                    std::cout << ros::Time::now() << std::endl;
                }
                if (last_length > -100.0)
                {
                    if (std::fabs(miran_sensor_data.length - last_length) < 50.0)
                    {
                        MiranSensorInfo_pub.publish(miran_sensor_data);
                        last_length = miran_sensor_data.length;
                    }
                }
                else
                {
                    last_length = miran_sensor_data.length;
                }
                std::cout << "===========================" << std::endl;
            }
            if (ser_count == 10)
            {
                ser_count = 0;
            }
        }
        else
        {
            std::cout << "ser.Open() is false" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
