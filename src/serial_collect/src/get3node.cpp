/*
 *  @author   : 闫治强
 *  @purpose  :串口直接订阅话题，然后调用回调函数发送到串口设备上。
 *  @time     :2017年08月08日14:26:28
 */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_srvs/Empty.h"
#include <impl/unix.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <tof_ros/Tofpoint.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "dji_sdk/MobileData.h"
#include "sensor_msgs/TimeReference.h"
#include <boost/thread.hpp>

using namespace std;
using namespace serial;
serial::Serial ser;
using std::string;
using std::vector;

geometry_msgs::Quaternion dji_current_atti;
sensor_msgs::BatteryState dji_msg_battery_state;
uint8_t dji_flight_status = 255;
uint8_t dji_display_mode  = 255;
sensor_msgs::NavSatFix    dji_current_gps;
sensor_msgs::Imu dji_synced_imu;
std_msgs::Float32 dji_agl_height;

class multiThreadListener
{
public:
    multiThreadListener()
    {
        sub1 = n.subscribe("/rplidar_scan", 1000, &multiThreadListener::chatterCallback_rplidar,this);//yes
        sub2 = n.subscribe("tof_scan", 100, &multiThreadListener::chatterCallback_toflidar,this);//yes
        sub3 = n.subscribe("dji_sdk/attitude", 10, &multiThreadListener::chatterCallback_djin3data_attitude,this);//FIND
        sub4 = n.subscribe("dji_sdk/battery_state", 10, &multiThreadListener::chatterCallback_djin3data_battery_state,this);
        sub5 = n.subscribe("dji_sdk/flight_status", 10, &multiThreadListener::chatterCallback_djin3data_flight_status,this);//FIND
        sub6 = n.subscribe("dji_sdk/gps_position", 10, &multiThreadListener::chatterCallback_djin3data_gps_position,this);//FIND
        sub7 = n.subscribe("dji_sdk/display_mode", 10, &multiThreadListener::chatterCallback_djin3data_display_mode,this);//FIND
        sub8 = n.subscribe("dji_sdk/imu", 10, &multiThreadListener::chatterCallback_djin3data_imu,this);
        sub9 = n.subscribe("dji_sdk/height_above_takeoff", 10, &multiThreadListener::chatterCallback_djin3data_height_above_takeoff,this);
//        sub10 = n.subscribe("dji_sdk/rc", 1, &multiThreadListener::chatterCallback_djin3data_rc,this);
//        sub11 = n.subscribe("dji_sdk/gps_health", 1, &multiThreadListener::chatterCallback_djin3data_gps_health,this);
//        sub12 = n.subscribe("dji_sdk/velocity", 1, &multiThreadListener::chatterCallback_djin3data_velocity,this);
//        sub13 = n.subscribe("dji_sdk/from_mobile_data", 1, &multiThreadListener::chatterCallback_djin3data_from_mobile_data,this);//FIND
//        sub14 = n.subscribe("dji_sdk/gimbal_angle", 1, &multiThreadListener::chatterCallback_djin3data_gimbal_angle,this);
//        sub15 = n.subscribe("dji_sdk/angular_velocity_fused", 1, &multiThreadListener::chatterCallback_djin3data_angular_velocity_fused,this);
//        sub16 = n.subscribe("dji_sdk/acceleration_ground_fused", 1, &multiThreadListener::chatterCallback_djin3data_acceleration_ground_fused,this);
//        sub17 = n.subscribe("dji_sdk/trigger_time", 1, &multiThreadListener::chatterCallback_djin3data_trigger_time,this);
    }
    void chatterCallback_rplidar(const sensor_msgs::LaserScan::ConstPtr& scan);
    void chatterCallback_toflidar(const tof_ros::Tofpoint::ConstPtr& msg);
    void chatterCallback_djin3data_attitude(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
    void chatterCallback_djin3data_battery_state(const sensor_msgs::BatteryState::ConstPtr& msg);
    void chatterCallback_djin3data_flight_status(const std_msgs::UInt8::ConstPtr& msg);
    void chatterCallback_djin3data_gps_position(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void chatterCallback_djin3data_height_above_takeoff(const std_msgs::Float32::ConstPtr& msg);
    void chatterCallback_djin3data_display_mode(const std_msgs::UInt8::ConstPtr& msg);
    void chatterCallback_djin3data_imu(const sensor_msgs::Imu::ConstPtr& msg);
//    void chatterCallback_djin3data_rc(const sensor_msgs::Joy::ConstPtr& msg);
//    void chatterCallback_djin3data_gps_health(const std_msgs::UInt8::ConstPtr& msg);
//    void chatterCallback_djin3data_velocity(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
//    void chatterCallback_djin3data_from_mobile_data(const dji_sdk::MobileData::ConstPtr& msg);
//    void chatterCallback_djin3data_gimbal_angle(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
//    void chatterCallback_djin3data_angular_velocity_fused(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
//    void chatterCallback_djin3data_acceleration_ground_fused(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
//    void chatterCallback_djin3data_trigger_time(const sensor_msgs::TimeReference::ConstPtr& msg);
public:
    ros::NodeHandle n;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Subscriber sub5;
    ros::Subscriber sub6;
    ros::Subscriber sub7;
    ros::Subscriber sub8;
    ros::Subscriber sub9;
//    ros::Subscriber sub10;
//    ros::Subscriber sub11;
//    ros::Subscriber sub12;
//    ros::Subscriber sub13;
//    ros::Subscriber sub14;
//    ros::Subscriber sub15;
//    ros::Subscriber sub16;
//    ros::Subscriber sub17;
//    ros::Subscriber sub18;

};

typedef unsigned char BYTE;
void multiThreadListener::chatterCallback_rplidar(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    uint16_t count = scan->scan_time / scan->time_increment;
    uint8_t intensitiesofu8[count]={0};
    for(int i = 0; i < count; i++)
    {
        intensitiesofu8[i]=(uint8_t)scan->intensities[i];
    }
    uint8_t bcnt=0;
    uint8_t check;
    uint16_t lenofvalid0=0;
    uint16_t lenofvalid1=0;
    uint16_t lenofvalid1_1=0;
    uint16_t lenofvalid2=0;
    uint16_t lenofvalid3=0;
    uint16_t lenofvalid4=0;
    uint64_t valid_start_time=(uint64_t)scan->header.stamp.toNSec() * 1e-3;
    lenofvalid0=sizeof(uint64_t);//起始时间
    lenofvalid1=sizeof(uint16_t);//距离
    lenofvalid1_1=count*lenofvalid1;
    lenofvalid2=count;//质量
    lenofvalid3=lenofvalid0+lenofvalid1_1+lenofvalid2;//有效长度
    lenofvalid4=lenofvalid3+9;
//*************************************************************
    BYTE b[lenofvalid4]={0};
    b[bcnt++]=(BYTE)0xAA;//默认１６进制数是ｉｎｔ型，转换为ｂｙｔｅ自动低８位。
    b[bcnt++]=(BYTE)0xAB;
    b[bcnt++]=(BYTE)0x00;//
    b[bcnt++]=(BYTE)0x01;
    b[bcnt++]=(BYTE)(3*count);//数据长度（有效）
    b[bcnt++]=(BYTE)((3*count)>>8);
    memcpy(&b[bcnt],&valid_start_time,lenofvalid0);
    int k=0;
    for(int j=0;j<count;j++)
    {
        memcpy(&b[bcnt+lenofvalid0+k+j*2],&scan->ranges[j],lenofvalid1);
        memcpy(&b[bcnt+lenofvalid0+lenofvalid1+k+j*2],&intensitiesofu8[j],1);
        k++;
    }
    for (int m = 6; m < (3*count+6); m++)
    {
        check += b[m];
    }
    memcpy(&b[lenofvalid3+6],&check,1);
    b[lenofvalid3+7]=0x5A;
    b[lenofvalid3+8]=0x5B;
//**********************************************************
//    string rplidata_show_start="This is rplidar_data start:";
//    ser.write(rplidata_show_start);
//    EnterCriticalSection（）;
    ser.write(b,lenofvalid4);
//    string rplidata_show_end="This is rplidar_data end";
//    ser.write(rplidata_show_end);


    ros::Rate loop_rate(10);//block chatterCallback2()
    loop_rate.sleep();
}

void multiThreadListener::chatterCallback_toflidar(const tof_ros::Tofpoint::ConstPtr& msg)
{
    int p=1;
    int ccnt=0;
    uint8_t tof_valid_chum=0x00;

    BYTE c[15]={0};
    c[ccnt++]=0xAA;
    c[ccnt++]=0xAB;
    c[ccnt++]=0x00;
    c[ccnt++]=0x02;
    c[ccnt++]=0x00;
    c[ccnt++]=0X06;
    c[ccnt++]=(uint8_t)(msg->delidar_distance);
    c[ccnt++]=(uint8_t)(msg->delidar_distance)>>8;
    c[ccnt++]=(uint8_t)(msg->delidar_strength);
    c[ccnt++]=(uint8_t)(msg->delidar_strength)>>8;
    c[ccnt++]=(uint8_t)(msg->delidar_signal_devel);
    c[ccnt++]=(uint8_t)(msg->delidar_time_exposed);
    for (int i_tof = 6; i_tof < 12; i_tof++)
    {
        tof_valid_chum += c[i_tof];
    }
    c[ccnt++]=tof_valid_chum;
    c[ccnt++]=0x5A;
    c[ccnt++]=0X5B;

//************************
//    string tofdata_show_start="This is tof_data start:";
//    ser.write(tofdata_show_start);
    ser.write(c,15);
//    string tofdata_show_end="This is tof_data end";
//    ser.write(tofdata_show_end);


    ros::Rate loop_rate(10);//block chatterCallback2()
    loop_rate.sleep();
}
//*******************************N3******************
void multiThreadListener::chatterCallback_djin3data_attitude(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  dji_current_atti = msg->quaternion;
  int dcnt=0;
  uint8_t dji_valid_chum=(BYTE)0x00;
  BYTE dji_data[250]={0};
  dji_data[dcnt++]=(BYTE)0xAA;
  dji_data[dcnt++]=(BYTE)0xAB;
  dji_data[dcnt++]=(BYTE)0x00;
  dji_data[dcnt++]=(BYTE)0x03;
  dji_data[dcnt++]=(BYTE)0x00;
  dji_data[dcnt++]=(BYTE)0X00;//now is 0x00,after to modify.
//attitude
  int len_atti=4*sizeof(double);
  memcpy(&dji_data[dcnt],&dji_current_atti,len_atti);
  int dji_datacnt=dcnt+len_atti;
  memcpy(&dji_data[dji_datacnt],&dji_flight_status,1);
  dji_datacnt=dji_datacnt+1;
//battery member
  int len_battery_capacity=sizeof(dji_msg_battery_state.capacity);
  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.capacity,len_battery_capacity);
  dji_datacnt=dji_datacnt+len_battery_capacity;

  int len_battery_voltage=sizeof(dji_msg_battery_state.voltage);
  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.voltage,len_battery_voltage);
  dji_datacnt=dji_datacnt+len_battery_voltage;

  int len_battery_current=sizeof(dji_msg_battery_state.current);
  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.current,len_battery_current);
  dji_datacnt=dji_datacnt+len_battery_current;

  int len_battery_percentage=sizeof(dji_msg_battery_state.percentage);
  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.percentage,len_battery_percentage);
  dji_datacnt=dji_datacnt+len_battery_percentage;

  int len_battery_charge=sizeof(dji_msg_battery_state.charge);
  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.charge,len_battery_charge);
  dji_datacnt=dji_datacnt+len_battery_charge;

  int len_battery_design_capacity=sizeof(dji_msg_battery_state.design_capacity);
  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.design_capacity,len_battery_design_capacity);
  dji_datacnt=dji_datacnt+len_battery_design_capacity;

  memcpy(&dji_data[dji_datacnt],&dji_msg_battery_state.power_supply_health,1);
  memcpy(&dji_data[dji_datacnt+1],&dji_msg_battery_state.power_supply_status,1);
  memcpy(&dji_data[dji_datacnt+2],&dji_msg_battery_state.power_supply_technology,1);
  memcpy(&dji_data[dji_datacnt+3],&dji_msg_battery_state.present,1);
//gps
  dji_datacnt=dji_datacnt+4;
  memcpy(&dji_data[dji_datacnt],&dji_current_gps.latitude,24);
  dji_datacnt=dji_datacnt+24;
//displaymode
  memcpy(&dji_data[dji_datacnt],&dji_display_mode,1);
  dji_datacnt=dji_datacnt+1;
//imu
  memcpy(&dji_data[dji_datacnt],&dji_synced_imu.angular_velocity,24);
  dji_datacnt=dji_datacnt+24;
  memcpy(&dji_data[dji_datacnt],&dji_synced_imu.linear_acceleration,24);
  dji_datacnt=dji_datacnt+24;
  memcpy(&dji_data[dji_datacnt],&dji_synced_imu.orientation,32);
  dji_datacnt=dji_datacnt+32;
//  height
  memcpy(&dji_data[dji_datacnt],&dji_agl_height.data,4);
  dji_datacnt=dji_datacnt+4;
  for (int i_dji = 6; i_dji <dji_datacnt ; i_dji++)
  {
      dji_valid_chum += dji_data[i_dji];
  }
  dji_data[4]=(BYTE)dji_datacnt;
  dji_data[5]=(BYTE)dji_datacnt>>8;
  dji_data[dji_datacnt++]=dji_valid_chum;
  dji_data[dji_datacnt++]=(BYTE)0x5A;
  dji_data[dji_datacnt++]=(BYTE)0X5B;
//*****************************8

  ser.write(dji_data,dji_datacnt);

  ros::Rate loop_rate(10);
  loop_rate.sleep();
}

void multiThreadListener::chatterCallback_djin3data_battery_state(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  dji_msg_battery_state.capacity = msg->capacity;
  dji_msg_battery_state.voltage = msg->voltage;
  dji_msg_battery_state.current = msg->current;
  dji_msg_battery_state.percentage = msg->percentage;
  dji_msg_battery_state.charge = msg->charge;
  dji_msg_battery_state.design_capacity = msg->design_capacity;
  dji_msg_battery_state.power_supply_health = msg->power_supply_health;
  dji_msg_battery_state.power_supply_status = msg->power_supply_status;
  dji_msg_battery_state.power_supply_technology = msg->power_supply_technology;
  dji_msg_battery_state.present = msg->present;
}
void multiThreadListener::chatterCallback_djin3data_flight_status(const std_msgs::UInt8::ConstPtr& msg)
{
  dji_flight_status = msg->data;
}
void multiThreadListener::chatterCallback_djin3data_gps_position(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  dji_current_gps = *msg;
}
void multiThreadListener::chatterCallback_djin3data_display_mode(const std_msgs::UInt8::ConstPtr& msg)
{
  dji_display_mode = msg->data;
}
void multiThreadListener::chatterCallback_djin3data_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  dji_synced_imu = *msg;
}
void multiThreadListener::chatterCallback_djin3data_height_above_takeoff(const std_msgs::Float32::ConstPtr& msg)
{
  dji_agl_height.data = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"get3node");//
    multiThreadListener listener_obj;
    ros::Publisher read_pub = listener_obj.n.advertise<std_msgs::String>("read",1000);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        ser.setBytesize(eightbits);
        ser.setParity(parity_none);
        ser.setStopbits(stopbits_one);
        serial::Timeout to = serial::Timeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

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
        ros::spinOnce();
        if(ser.available())
        {
            ROS_INFO_STREAM("reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());//duquchuankou data
            ROS_INFO_STREAM("read:"<< result.data);
            read_pub.publish(result);//发送消息到话题的实现的地方
        }
        loop_rate.sleep();
    }
   ros::waitForShutdown();
   return 0;

}
