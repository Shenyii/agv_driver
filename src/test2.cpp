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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define FALSE -1  
#define TRUE 0

double dtime = 0.0;
double left_vel=0,right_vel=0;
geometry_msgs::Twist cmd_vel;
unsigned char firstrecvflag;  //first recieve init odometry
int speed_arr[9] = {B500000, B38400, B19200, B115200, B9600, B4800, B2400, B1200, B300 };
int name_arr[9]  = {500000, 38400,  19200, 115200, 9600,  4800,  2400,  1200,  300 };
double velTime = 0.0;  //receive cmd_vel time
int UartOpenFlag = 0;
int left_dis=0;
int right_dis=0;
int left_stop=0;
int right_stop=0;
int count_false=0;
long Lcode_temp0,Rcode_temp0,Lcode_temp1,Rcode_temp1;
short imu_init=0;

struct Opose{
      double x;
      double y;
      double heading[4];
    };

class Odometry{
  public:
    Odometry();
    void PoseMove(const int encodeL,const int encodeR,const int IMU);
    geometry_msgs::Twist TwistMove(geometry_msgs::Pose movepose);
    void update(geometry_msgs::Pose movepose, geometry_msgs::Twist movetwist);
    void publishTransform(const geometry_msgs::Quaternion &odom_quat);
    void publishOdometry(const geometry_msgs::Quaternion &odom_quat, geometry_msgs::Twist movetwist);
    void PoseTranslation(const double dis, const double angle, Opose &poseForward);
    void Pose_Update(const Opose forward);
	
    //encode
    void encode_PoseMove(const int encodeL,const int encodeR,const int IMU);
    void encode_Pose_Update(const Opose forward);
    void encode_publishTransform(const geometry_msgs::Quaternion &odom_quat);
    void encode_publishOdometry(const geometry_msgs::Quaternion &odom_quat, geometry_msgs::Twist movetwist);

  private:
    double move_dis;
    double move_angle;
    double updateDis;
    double updateAngle;
	
	double move_angle_encode;

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    ros::NodeHandle nh;
    ros::Publisher odom_publisher; 
	
    //encode
    ros::Publisher encode_odom_publisher;
    tf::TransformBroadcaster odom_broadcaster_encode;
    geometry_msgs::TransformStamped odom_trans_encode;
  protected: 

};

const    int speed_ratio = 4096;      //4096*4;
const double wheel_perimeter = 53.4;   //cm
const double bias = 30;               //cm
const double frequency_encode = 10;   //ignore it, useless
int  debugFlag = 0;
bool use_imu_heading = true;
Opose poseUpdate;
Opose poseUpdate_encode;

Odometry::Odometry()
{
  move_dis   = 0;
  move_angle = 0;
  updateDis  = 0;
  updateAngle = 0;

  poseUpdate.x = 0.0;
  poseUpdate.y = 0.0;
  poseUpdate.heading[0] = 1.0;  //cos
  poseUpdate.heading[1] = 0.0;  //-sin
  poseUpdate.heading[2] = 0.0;  //sin
  poseUpdate.heading[3] = 1.0;  //cos
  
  poseUpdate_encode.x = 0.0;
  poseUpdate_encode.y = 0.0;
  poseUpdate_encode.heading[0] = 1.0;  //cos
  poseUpdate_encode.heading[1] = 0.0;  //-sin
  poseUpdate_encode.heading[2] = 0.0;  //sin
  poseUpdate_encode.heading[3] = 1.0;  //cos

  odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 100);
  encode_odom_publisher = nh.advertise<nav_msgs::Odometry>("odom_encode", 100);
}

double LeftVel,RightVel;
double minVelL=100,maxVelL=0,avgVelL=0,avrL=0,sumVelL=0;
double minVelR=100,maxVelR=0,avgVelR=0,avrR=0,sumVelR=0;
double velStrL[2000],velStrR[2000];
double sumtime=0;
int countvel=0;
double varianceL=0.0,varianceR=0.0;
void Odometry::PoseMove(const int encodeL, const int encodeR, const int IMU)
{
  ///printf("encodeL=%d,encodeR=%d.\n",encodeL,encodeR);
  Opose poseForward;
  geometry_msgs::Pose movepose;
  move_dis = wheel_perimeter*(encodeL + encodeR)/2.0/speed_ratio;
  move_angle_encode = wheel_perimeter*(encodeR - encodeL)/speed_ratio/bias;
  if(use_imu_heading == true)
  {
    move_angle = IMU* 0.0174532925;  //3.1415926/180;
  }

  //printf("dis=%f,IMU=%d.\n",move_dis,IMU);
  PoseTranslation(move_dis,move_angle,poseForward);  
  geometry_msgs::Twist movetwist;
 
  movetwist.linear.x = poseForward.x/dtime;//frequency_encode;
  movetwist.angular.z = move_angle_encode/dtime;//frequency_encode;
  Pose_Update(poseForward);
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(std::atan2(poseForward.heading[2],poseForward.heading[0]));
  publishTransform(odom_quat);
  publishOdometry(odom_quat,movetwist);  
  
  LeftVel  = wheel_perimeter*(encodeL)/speed_ratio/dtime;
  RightVel = wheel_perimeter*(encodeR)/speed_ratio/dtime;
  sumtime += dtime;
  if(sumtime > 3 && sumtime < 15)
  {
    if(LeftVel < minVelL)
    {
      minVelL = LeftVel;
    }
    if(LeftVel > maxVelL)
    {
      maxVelL = LeftVel;
    }
    sumVelL += LeftVel;
    velStrL[countvel] = LeftVel;
    
    if(RightVel < minVelR)
    {
      minVelR = RightVel;
    }
    if(RightVel > maxVelR)
    {
      maxVelR = RightVel;
    }
    sumVelR += RightVel;
    velStrR[countvel] = RightVel;
    if(countvel > 2000)
    {
      sumtime = 100;
    }
    countvel++;
  }
  if(sumtime >= 15)
  {
    avgVelL = sumVelL/countvel;
    avgVelR = sumVelR/countvel;
    varianceL = varianceR = 0;
    for(int i=0; i<countvel; i++)
    {
      varianceL += (velStrL[i]-avgVelL)*(velStrL[i]-avgVelL);
      varianceR += (velStrR[i]-avgVelR)*(velStrR[i]-avgVelR);
    }
    avrL = varianceL/countvel;
    avrR = varianceR/countvel;
  }
  if(debugFlag == 1)
  {
    //printf("drivervel,leftvel=%f,rightvel=%f,leftrate=%f,rightrate=%f\n",wheel_perimeter*(encodeL)/speed_ratio/dtime,wheel_perimeter*(encodeR)/speed_ratio/dtime,wheel_perimeter*(encodeL)/speed_ratio/dtime/left_vel-1,wheel_perimeter*(encodeR)/speed_ratio/dtime/right_vel-1);
    printf("poseupdate,x=%f,y=%f,x/y=%f,y/x=%f.\n",poseUpdate.x,poseUpdate.y,poseUpdate.x/poseUpdate.y,poseUpdate.y/poseUpdate.x);
    printf("Left:min=%f,max=%f,avg=%f,variance=%f\n",minVelL,maxVelL,avgVelL,avrL);
    printf("Right:min=%f,max=%f,avg=%f,variance=%f\n",minVelR,maxVelR,avgVelR,avrR);
  }
}

geometry_msgs::Twist Odometry::TwistMove(geometry_msgs::Pose movepose)
{
  geometry_msgs::Twist movetwist;
 // movetwist.linear.x = movepose.position.x/frequency_encode;
 // movetwist.angular.z = move_angle/frequency_encode;
  return movetwist;
}

//tf::TransformBroadcaster odom_broadcaster;
//geometry_msgs::TransformStamped odom_trans;

void Odometry::publishTransform(const geometry_msgs::Quaternion &odom_quat)
{
  odom_trans.header.stamp    = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "base_footprint";
  odom_trans.transform.translation.x = poseUpdate.x/100.0;
  odom_trans.transform.translation.y = poseUpdate.y/100.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);
}

void Odometry::publishOdometry(const geometry_msgs::Quaternion &odom_quat, geometry_msgs::Twist movetwist)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = poseUpdate.x/100;
  odom.pose.pose.position.y = poseUpdate.y/100;
  odom.pose.pose.position.z = move_angle*180/3.1415926; //0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x  = movetwist.linear.x/100;
  odom.twist.twist.linear.y  = 0.0;
  odom.twist.twist.angular.z = movetwist.angular.z;

  odom.pose.covariance[0]  = 0.1;
  odom.pose.covariance[7]  = 0.1;
  odom.pose.covariance[35] = 0.05; //use_imu_heading ? 0.05 : 0.2;

  odom.pose.covariance[14] = DBL_MAX; // set a non-zero covariance on unused
  odom.pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this
  odom.pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf

  odom_publisher.publish(odom);
}
void Odometry::update(geometry_msgs::Pose movepose, geometry_msgs::Twist movetwist)
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(updateAngle);
  publishTransform(odom_quat);
  publishOdometry(odom_quat,movetwist);    
}

void Odometry::PoseTranslation(const double dis, const double angle, Opose &poseForward)
{
  double c = std::cos(angle);
  double s = std::sin(angle);
  poseForward.x = dis;
  poseForward.y = 0.0;
  poseForward.heading[0] = c;
  poseForward.heading[1] = -s;
  poseForward.heading[2] = s;
  poseForward.heading[3] = c;
  //printf("heading1=%f,2=%f,3=%f,4=%f\n",poseForward.heading[0],poseForward.heading[1],poseForward.heading[2],poseForward.heading[3]);
}

void Odometry::Pose_Update(const Opose forward)
{
  /*poseUpdate.heading[0] = poseUpdate.heading[0]*forward.heading[0]
                         +poseUpdate.heading[1]*forward.heading[2];
  poseUpdate.heading[1] = poseUpdate.heading[0]*forward.heading[1]
                         +poseUpdate.heading[1]*forward.heading[3];
  poseUpdate.heading[2] = poseUpdate.heading[2]*forward.heading[0]
                         +poseUpdate.heading[3]*forward.heading[2];
  poseUpdate.heading[3] = poseUpdate.heading[2]*forward.heading[1]
                         +poseUpdate.heading[3]*forward.heading[3];*/

  poseUpdate.x += forward.heading[0]*forward.x
                 +forward.heading[1]*forward.y;
  poseUpdate.y += forward.heading[2]*forward.x
                 +forward.heading[3]*forward.y; 

  //printf("heading=%f\n",(atan2(poseUpdate.heading[2],poseUpdate.heading[0]))/3.14159*180);
}


//encode
void Odometry::encode_PoseMove(const int encodeL, const int encodeR, const int IMU)
{
  ///printf("encodeL=%d,encodeR=%d.\n",encodeL,encodeR);
  Opose poseForward;
  geometry_msgs::Pose movepose;
  move_dis = wheel_perimeter*(encodeL + encodeR)/2.0/speed_ratio;
  move_angle = wheel_perimeter*(encodeR - encodeL)/speed_ratio/bias;

  //printf("dis=%f,IMU=%d.\n",move_dis,IMU);
  PoseTranslation(move_dis,move_angle,poseForward);  
  geometry_msgs::Twist movetwist;
 
  movetwist.linear.x = poseForward.x/dtime;//frequency_encode;
  movetwist.angular.z = move_angle/dtime;//frequency_encode;
  encode_Pose_Update(poseForward);
  geometry_msgs::Quaternion odom_quat1 = tf::createQuaternionMsgFromYaw(std::atan2(poseUpdate_encode.heading[2],poseUpdate_encode.heading[0]));
  //encode_publishTransform(odom_quat1);
  encode_publishOdometry(odom_quat1,movetwist); 
}
void Odometry::encode_Pose_Update(const Opose forward)
{
  poseUpdate_encode.heading[0] = poseUpdate_encode.heading[0]*forward.heading[0]
                                +poseUpdate_encode.heading[1]*forward.heading[2];
  poseUpdate_encode.heading[1] = poseUpdate_encode.heading[0]*forward.heading[1]
                                +poseUpdate_encode.heading[1]*forward.heading[3];
  poseUpdate_encode.heading[2] = poseUpdate_encode.heading[2]*forward.heading[0]
                                +poseUpdate_encode.heading[3]*forward.heading[2];
  poseUpdate_encode.heading[3] = poseUpdate_encode.heading[2]*forward.heading[1]
                                +poseUpdate_encode.heading[3]*forward.heading[3];

  poseUpdate_encode.x += poseUpdate_encode.heading[0]*forward.x
                        +poseUpdate_encode.heading[1]*forward.y;
  poseUpdate_encode.y += poseUpdate_encode.heading[2]*forward.x
                        +poseUpdate_encode.heading[3]*forward.y; 

  //printf("heading=%f\n",(atan2(poseUpdate.heading[2],poseUpdate.heading[0]))/3.14159*180);
}
void Odometry::encode_publishTransform(const geometry_msgs::Quaternion &odom_quat)
{
  odom_trans_encode.header.stamp = ros::Time::now();
  odom_trans_encode.header.frame_id = "odom_encode";
  odom_trans_encode.child_frame_id  = "base_footprint";
  odom_trans_encode.transform.translation.x = poseUpdate_encode.x/100.0;
  odom_trans_encode.transform.translation.y = poseUpdate_encode.y/100.0;
  odom_trans_encode.transform.translation.z = 0.0;
  odom_trans_encode.transform.rotation = odom_quat;
  odom_broadcaster_encode.sendTransform(odom_trans_encode);
}
void Odometry::encode_publishOdometry(const geometry_msgs::Quaternion &odom_quat, geometry_msgs::Twist movetwist)
{
  nav_msgs::Odometry odom_encode;
  odom_encode.header.stamp = ros::Time::now();
  odom_encode.header.frame_id = "odom_encode";

  odom_encode.pose.pose.position.x = poseUpdate_encode.x/100;
  odom_encode.pose.pose.position.y = poseUpdate_encode.y/100;
  odom_encode.pose.pose.position.z = tf::getYaw(odom_quat)*180/3.1415926; //0.0;
  odom_encode.pose.pose.orientation = odom_quat;

  odom_encode.child_frame_id = "base_footprint";
  odom_encode.twist.twist.linear.x = movetwist.linear.x/100;
  odom_encode.twist.twist.linear.y = 0.0;
  odom_encode.twist.twist.angular.z = movetwist.angular.z;

  odom_encode.pose.covariance[0]  = 0.1;
  odom_encode.pose.covariance[7]  = 0.1;
  odom_encode.pose.covariance[35] = 0.2; //use_imu_heading ? 0.05 : 0.2;

  odom_encode.pose.covariance[14] = DBL_MAX; // set a non-zero covariance on unused
  odom_encode.pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this
  odom_encode.pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf

  encode_odom_publisher.publish(odom_encode);
}

#ifndef _UART_COMM_H_
#define _UART_COMM_H_

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
  protected:
  private:
    double vel_x;
    double vel_th;
    double width_robot;
    unsigned char firstrecvflag;
};




BaseData basedata;


UARTComm::UARTComm()
{
  
}

void UARTComm::set_speed(int fd,int speed)
{
  int   i;
  int   status;
  struct termios   Opt;
  tcgetattr(fd, &Opt);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
  {
    if (speed == name_arr[i]) 
    {
      tcflush(fd, TCIOFLUSH);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if (status != 0) 
      {
        perror("tcsetattr fd1");
        return;
      }
      tcflush(fd,TCIOFLUSH);
    }
  }
}

int UARTComm::set_Parity(int fd,int databits,int stopbits,int parity)
{
  struct termios options;   
  if ( tcgetattr(fd,&options) != 0) 
  {   
    perror("SetupSerial 1");       
    return(FALSE);    
  }  
  options.c_cflag &= ~CSIZE;   
  switch (databits)   
  {     
    case 7:       
        options.c_cflag |= CS7;   
        break;  
    case 8:       
        options.c_cflag |= CS8;  
        break;     
    default:      
        fprintf(stderr,"Unsupported data size\n"); return (FALSE);    
  }  
  switch (parity)   
  {     
    case 'n':  
    case 'N':      
        options.c_cflag &= ~PARENB;   /* Clear parity enable */  
        options.c_iflag &= ~INPCK;    /* Enable parity checking */   
        break;    
    case 'o':     
    case 'O':       
        options.c_cflag |= (PARODD | PARENB);   
        options.c_iflag |= INPCK;     /* Disnable parity checking */   
        break;    
    case 'e':    
    case 'E':     
        options.c_cflag |= PARENB;    /* Enable parity */      
        options.c_cflag &= ~PARODD;      
        options.c_iflag |= INPCK;     /* Disnable parity checking */  
        break;  
    case 'S':   
    case 's':                         /*as no parity*/     
        options.c_cflag &= ~PARENB;  
        options.c_cflag &= ~CSTOPB;break;    
    default:     
        fprintf(stderr,"Unsupported parity\n");      
        return (FALSE);    
  }    
      
  switch (stopbits)  
  {     
    case 1:      
        options.c_cflag &= ~CSTOPB;    
        break;    
    case 2:      
        options.c_cflag |= CSTOPB;    
        break;  
    default:      
        fprintf(stderr,"Unsupported stop bits\n");    
        return (FALSE);   
  }   
  /* Set input parity option */   
  if (parity != 'n')     
      options.c_iflag |= INPCK;

  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag  &= ~OPOST;   /*Output*/
  options.c_iflag &= ~(ICRNL | IXON);
   
  tcflush(fd,TCIFLUSH);  
  options.c_cc[VTIME] = 10;   
  options.c_cc[VMIN]  = 14;     /* Update the options and do it NOW */  
  if (tcsetattr(fd,TCSANOW,&options) != 0)     
  {   
    perror("SetupSerial 3");     
    return (FALSE);    
  }   
  return (TRUE); 
}


int UARTComm::RecvBaseBuf(unsigned char *rbuf)
{

  int i=0;
  unsigned char checksum = 0;
  //debug output 
  //printf("rbuf=");
  //for(i=0;i<22;i++)printf("%d ",rbuf[i]);
  //printf("\n");

  if(rbuf == NULL)
  {
    count_false++;
    printf("receive buf is null!\n");
    return 0;
  }
  else
  {
    if(((rbuf[0]&0xff) != 0x23) || ((rbuf[21]&0xff) != 0x32))
    {
      count_false++;
      printf("buf head or tail fault! buf[0]=%x,buf[21]=%x\n",rbuf[0],rbuf[22]);
      return 0;
    }
    
    for(int i=1; i<20; i++)
    {
      checksum += rbuf[i];
    }
    
    if((rbuf[20]&0xff) != (checksum & 0xff))
    {
      count_false ++;
      printf("checksum fault!=%x\n",(checksum &0xff));
      return 0;
    }
	
    //ultrasonic [1-6]
    basedata.ultrasonic1 = rbuf[1]&0xff;
    basedata.ultrasonic2 = rbuf[2]&0xff;
    basedata.ultrasonic3 = rbuf[3]&0xff;
    basedata.ultrasonic4 = rbuf[4]&0xff;
    basedata.ultrasonic5 = rbuf[5]&0xff;
    basedata.ultrasonic6 = rbuf[6]&0xff;

    //bumper [7]
    basedata.bump1   = (rbuf[7]>>7) & 0x01;
    basedata.bump2   = (rbuf[7]>>6) & 0x01;
    basedata.bump3   = (rbuf[7]>>5) & 0x01;
    basedata.bump4   = (rbuf[7]>>4) & 0x01;
    basedata.bump5   = (rbuf[7]>>3) & 0x01;
    basedata.bump6   = (rbuf[7]>>2) & 0x01;
    basedata.Lcode_  = (rbuf[7]>>1) & 0x01;
    basedata.Rcode_  = (rbuf[7]>>0) & 0x01;

    //battery [8]
    basedata.battery = rbuf[8]&0xff;
	
    //encode [9-16]
    Lcode_temp0 = Lcode_temp1;
    Lcode_temp1 = ((rbuf[9]<<24)&0xff000000) | ((rbuf[10]<<16) & 0xff0000) | ((rbuf[11]<<8)&0xff00) | ((rbuf[12]) & 0xff);
    if(firstrecvflag==2) basedata.Lcode = Lcode_temp1-Lcode_temp0;
    Rcode_temp0 = Rcode_temp1;
    Rcode_temp1 = ((rbuf[13]<<24)&0xff000000) | ((rbuf[14]<<16) & 0xff0000) | ((rbuf[15]<<8)&0xff00) | ((rbuf[16]) & 0xff);
    if(firstrecvflag==2) basedata.Rcode = Rcode_temp0-Rcode_temp1;
     
    //IMU [17-18]
    if(firstrecvflag==0)
    {
       imu_init = (((rbuf[17]<<8) & 0xff00) | ((rbuf[18]) & 0xff));
       basedata.IMU = 0;
    }
    if(firstrecvflag!=0)
       basedata.IMU = (((rbuf[17]<<8) & 0xff00) | ((rbuf[18]) & 0xff)) - imu_init;
    
    //base_state [19]
    basedata.base_state  = rbuf[19]&0xff;
 
    //printf("left_dis=%f,right_dis=%f,Lcode=%d,Rcode=%d\n",(left_dis/100.0),(right_dis/100.0),basedata.Lcode,basedata.Rcode);
    //printf("ult1=%d,ult2=%d,ult3=%d,IMU=%d,Lcode=%d,Rcode=%d,state1=%d,%d,%d,%d,%d,%d,%d,%d,state2=%d,%d,%d,%d,%d,%d\n",basedata.ultrasonic1,basedata.ultrasonic2,basedata.ultrasonic3,basedata.IMU,basedata.Lcode,basedata.Rcode,basedata.wheelstate,basedata.occupystate,basedata.chargestate,basedata.lowbatterystate,basedata.ultstate3,basedata.ultstate2,basedata.ultstate1,basedata.infrared,basedata.fallstate5,basedata.fallstate4,basedata.fallstate3,basedata.fallstate2,basedata.fallstate1,basedata.all3state);
    
  }
  return 1;
}

void UARTComm::SendBuf(unsigned char *sbuf,bool flag,short Lvel,short Rvel)
{
  unsigned char checksum = 0;
  //Lvel = Lvel*1000;
  //Rvel = Rvel*1000;
  sbuf[0] = 0x23;
  sbuf[1] = (Lvel>> 8)&0xFF;
  sbuf[2] = Lvel&0xFF;
  sbuf[3] = (Rvel>> 8)&0xFF;
  sbuf[4] = Rvel&0xFF;
 
  for(int i=1; i<5; i++)
  {
    checksum += sbuf[i];
  }
  sbuf[5] = checksum & 0xff;
  sbuf[6] = 0x32;
  //printf("\n lvel=%d,rvel=%d \r\n",Lvel,Rvel);
}

void UARTComm::UARTComm_CallBack(geometry_msgs::Twist msg)
{
    //printf("UARTComm_CallBack!\n");
    vel_x  = msg.linear.x;  //last_cmd_vel.linear.x / 1.0; //vel ratio
    vel_th = msg.angular.z;  //last_cmd_vel.angular.z;
  
    vel_x = 1000 * vel_x;
    if(vel_x == 0)
    {
      right_vel = vel_th * width_robot / 2.0;
      left_vel = (-1) * right_vel;
    }
    else if(vel_th == 0)
    {
      left_vel = right_vel = vel_x;
    }
    else
    {
      left_vel = vel_x - vel_th * width_robot / 2.0;
      right_vel = vel_x + vel_th * width_robot / 2.0;
    }

}


void UARTComm::Uart_Main()
{
  int fd;
  int len,fs_sel;
  fd_set fs_read;
  struct timeval time;
  int recvflag = 0;
  unsigned int IMU_Init = 0;
  int IMU_send = 0;
  int timeout = 0;
  unsigned char rbuff[22];   
  int  nread;    
  unsigned char sbuf[5];
  int  retv;
  int  length;  
  double sumtime = 0;
  firstrecvflag = 0;
  Odometry odom;

  printf("open /dev/agv_driver...\n");
  while(timeout<30) //almost 3s break
  {  
    //fd = open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY); 
    fd = open("/dev/agv_driver",O_RDWR|O_NOCTTY|O_NDELAY); 
    usleep(1000*100); 
    if(fd == -1)  
    {        
      printf("\033[31mopen serialport error \033[0m !!!\n");
      sleep(1);
      timeout+=10;
      continue;
    }  
    else  
    {  
      printf("%s",ttyname(fd));  
      printf(" succesfully\n");   
      timeout =100;
      break;
    } 
    timeout++; 
    if(timeout==30)printf("\033[31mError, operation time out. \033[0m !!!\n");
  }
  timeout=0;

  set_speed(fd,115200);  
  if (set_Parity(fd,8,1,'N') == FALSE)  
  {  
    printf("Set Parity Error\n");  
    exit (0);  
  }  

  
  if(fcntl(fd,F_SETFL,0) < 0)
  {     
    printf("fcntl failed\n");     
  } 
  pthread_mutex_t rMutex = PTHREAD_MUTEX_INITIALIZER;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  UartOpenFlag = 1;


  ros::NodeHandle n;
  n.param("width_robot", width_robot ,351.4); 
  ros::Publisher  recv_pub = n.advertise<geometry_msgs::Twist>("recv_vel",100);
  ros::Subscriber  vel_sub = n.subscribe("cmd_vel", 1000, &UARTComm::UARTComm_CallBack, this); 
  geometry_msgs::Twist msg;

  //init odom
  odom.PoseMove(0,0,0);  
  odom.encode_PoseMove(0,0,0);

  ros::Rate loop_rate(20);
  while (ros::ok()) 
  {  
    //usleep(1000*9);
    tcflush(fd,TCIFLUSH);
    tcflush(fd,TCOFLUSH);

    //printf("uart_comm running: left_vel=%f,right_vel=%f\n",left_vel,right_vel);
    //SendBuf(sbuf,01,10,10);
    //printf("\nuartvel=%f,rvel=%f\n",left_vel,right_vel);
    //if(velTime < 2)
    SendBuf(sbuf,01,left_vel,right_vel);
    //else
    //SendBuf(sbuf,01,0.0,0.0);

    msg.linear.x  = left_vel/100.0;
    msg.angular.z = right_vel/100.0;
    recv_pub.publish(msg);
    //printf("\n");
    //for(int i=0;i<6;i++)
    //printf("%x ", sbuf[i]);
    //printf("\n");
    bzero(rbuff, sizeof(rbuff));
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec  = 5;
    time.tv_usec = 0;

    //data send
    length = sizeof(sbuf);
    //retv = write(fd, sbuf, length);
    retv = write(fd, sbuf, 7);
    if(retv == -1)
    {
      perror("Write data error!\n");
    } 
   
    if((nread = read(fd, rbuff, 22))>0)
    { 
      last_time = ros::Time::now();
      dtime = (last_time - current_time).toSec();
      sumtime += dtime;
      velTime += dtime;
      //printf("recv sumtime = %f.\n",sumtime); 
      current_time = ros::Time::now();
      //for(int i=0;i<nread;i++)printf(" %x",rbuff[i]&0xff);printf("\n"); 
 
      recvflag = RecvBaseBuf(rbuff);
      if(recvflag == 1)
      { 
        firstrecvflag++;
        if(firstrecvflag>2) firstrecvflag=2;
        IMU_send=basedata.IMU/10;
        odom.PoseMove(basedata.Lcode,basedata.Rcode,IMU_send);
        odom.encode_PoseMove(basedata.Lcode,basedata.Rcode,IMU_send);
        printf("IMU=%d,left_vel=%f,right_vel=%fmm/s\n",IMU_send,left_vel,right_vel);
      }
      else
      {
	 odom.PoseMove(basedata.Lcode,basedata.Rcode,IMU_send);
	 printf("read uart data fault count_false=%d!\n",count_false);
      }

    }
    else
    {
       printf("read uart data timeout 1s!\n");
    }

    ros::spinOnce();
    loop_rate.sleep();
    
  }//end while  
  close(fd); 
}

int main(int argc,char** argv)
{
    int data[7] = {0x23,0x55,0x55,0x55,0x55,0x54,0x32};
    ros::init(argc,argv,"test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",5);
    ros::Rate loop_rate(0.5);

    UARTComm serial_test;
    left_vel = 5;
    right_vel = 5;
    
    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "ok" << 5;
        msg.data = ss.str();
        chatter_pub.publish(msg);
        serial_test.Uart_Main();
        ROS_INFO("OK");
        loop_rate.sleep();
    }
}