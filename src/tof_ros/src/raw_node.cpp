#include <ros/ros.h>
#include "serial.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tof_ros/Tofpoint.h>
using namespace serial;
serial::Serial ser;

int main (int argc,char**argv)
{
    ros::init(argc,argv,"raw_node");//
    ros::NodeHandle nh;
    //发布主题to listener from other device's data
    ros::Publisher read_pub = nh.advertise<tof_ros::Tofpoint>("tof_scan",100);

    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB2");//前面我们配置了雷达的usb转串口为ttyusb0
                                      //  我们在加一个就是ttyusb1
        ser.setBaudrate(115200);//我们s和另一个设备的波特率
        serial::Timeout to = serial::Timeout(1000);//通讯是的延时时间，每一个字节的
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("serial port innitialized");
    }
    else
    {
        return -1;
    }
    //指定循环的频率
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        //处理ROS的信息，比如订阅消息,并调用回调函数
        //在这个函数调用时处理回调函数，它在死循环下，
        //也就说操作系统不停的判断它来判断是否所订阅的话题有了新的消息

        uint8_t tofnodesBuffer_RXD[9]={0};
        uint16_t distance = 0;
        uint16_t strength = 0;
        uint8_t tof_chum=0;
        uint8_t cha;
        ros::spinOnce();

        while (ser.available())
        {

            tof_ros::Tofpoint result;

            ROS_INFO_STREAM("reading from serial port");
            for (int i = 0; i < 9; i++)
            {
                ser.read(&cha,1);
                tofnodesBuffer_RXD[i] =cha;
                ROS_INFO_STREAM("read:"<< tofnodesBuffer_RXD[i]);
            }

            tof_chum = 0;

            for (int i = 0; i < 8; i++)
            {
                tof_chum += tofnodesBuffer_RXD[i];
            }

            if (tof_chum == tofnodesBuffer_RXD[8])
            {
                strength = (tofnodesBuffer_RXD[5] << 8) + tofnodesBuffer_RXD[4];
                distance = ( tofnodesBuffer_RXD[3] << 8 ) + tofnodesBuffer_RXD[2];
                result.delidar_frame_start1=tofnodesBuffer_RXD[0];
                result.delidar_frame_start2=tofnodesBuffer_RXD[1];
                result.delidar_distance=distance;
                result.delidar_strength=strength;
                result.delidar_signal_devel=tofnodesBuffer_RXD[6];
                result.delidar_time_exposed=tofnodesBuffer_RXD[7];
                result.delidar_checksum=tofnodesBuffer_RXD[8];
            }
            read_pub.publish(result);//发送消息到话题的实现的地方
         }

        loop_rate.sleep();//在死循环里面的结尾处，意思是这个程序里面的要慢点。
    }


}
