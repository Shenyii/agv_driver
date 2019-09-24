#include <setjmp.h>
#include <signal.h>
#include "ros/ros.h"
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "uart_comm.h"
#include "odometry.h"

extern double left_vel;
extern double right_vel;

int main(int argc,char** argv)
{
    int data[7] = {0x23,0x55,0x55,0x55,0x55,0x54,0x32};
    ros::init(argc,argv,"test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",5);
    ros::Rate loop_rate(0.5);

    //UARTComm serial_test;
    
    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "ok" << 5;
        msg.data = ss.str();
        chatter_pub.publish(msg);
        //serial_test.Uart_Main();
        ROS_INFO("%f",cos(3.1415926));
        loop_rate.sleep();
    }
}