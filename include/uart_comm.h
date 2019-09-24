#include<ros/ros.h>
#include<stdio.h>      
#include<stdlib.h>     
#include<unistd.h>       
#include<sys/types.h>   
#include<sys/stat.h>     
#include<fcntl.h>        
#include<termios.h>      
#include<errno.h>       
#include<string.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int8.h"

#ifndef _UART_COMM_H_
#define _UART_COMM_H_

extern double velTime;  //receive cmd_vel time
extern double dtime;  //receive uart message time
extern int UartOpenFlag;
extern double left_vel;
extern double right_vel;
struct BaseData{
  unsigned char ultrasonic1;
  unsigned char ultrasonic2;
  unsigned char ultrasonic3;
  unsigned char ultrasonic4;
  unsigned char ultrasonic5;
  unsigned char ultrasonic6;
 
  bool bump1;
  bool bump2;
  bool bump3;
  bool bump4;
  bool bump5;
  bool bump6;
  bool Lcode_;
  bool Rcode_;

  unsigned char battery;

  int Lcode;
  int Rcode;
  short IMU;
  
  unsigned char base_state;
};  //driver data from uart
#endif

class UARTComm{
  public:
    UARTComm();
    void set_speed(int fd, int speed);
    int set_Parity(int fd,int databits,int stopbits,int parity);
    int RecvBaseBuf(unsigned char *rbuf);
    void SendBuf(unsigned char *sbuf,bool flag,short Lvel,short Rvel);
    void UARTComm_CallBack(geometry_msgs::Twist msg);
    void Uart_Main();
  
  private:
    double vel_x;
    double vel_th;
    double width_robot;
    unsigned char firstrecvflag;

    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub_camera_pose_;
};
