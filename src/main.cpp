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
using namespace std;
pthread_mutex_t hMutex = PTHREAD_MUTEX_INITIALIZER;
float width_robot=35;
/*
float right_vel = 0;
float left_vel  = 0;
*/
double vel_x;
double vel_th;

unsigned int IMU_data = 0;

void CmdVel_CallBack(geometry_msgs::Twist msg)
{
    //printf("CmdVel_CallBack!\n");
    vel_x  = msg.linear.x; 
    vel_th = msg.angular.z;  
  
    if(vel_x == 0)
    {
      right_vel = vel_th * width_robot / 2.0;
      left_vel  = (-1) * right_vel;
    }
    else if(vel_th == 0)
    {
      left_vel = right_vel = vel_x;
    }
    else
    {
      left_vel  = vel_x - vel_th * width_robot / 2.0;
      right_vel = vel_x + vel_th * width_robot / 2.0;
    }

}

void *Thread_topic(void *)
{
  ros::NodeHandle tn;
  ros::Publisher rawvel_pub = tn.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  geometry_msgs::Twist msg;
  int j=0;  //for debug
  int count=0;
  ros::Rate loop_rate(10);
  while (ros::ok()) 
  {
    count++;
    //printf("%d Thread_topic run 10 hz...\n",count);
    //ROS_INFO("[%d]Thread_topic run 10 hz...",count);
    j++;
    if(j>20)
    {
      j=20;
      msg.linear.x = j;
    }
    else
    {
      msg.linear.x = j;
    }
    msg.linear.y  = 0;
    msg.linear.z  = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
     
    //rawvel_pub.publish(msg);
    loop_rate.sleep();
  } 
}
void Pthread_topic()
{
  pthread_t pid_topic;
  printf("usart thread start...\n");
  int pthread_spin = pthread_create(&pid_topic, NULL,Thread_topic, NULL);
}
 

void *Thread_usart(void *)
{
  UARTComm uartcomm;
  uartcomm.Uart_Main();
  /*
  printf("run usart\n");
  ros::NodeHandle un;
  int count = 0;
  ros::Rate loop_rate(10);
  while (ros::ok()) 
  {
    count++;
    //printf("%d Thread_usart run 10 hz...\n",count);
    ROS_INFO("[%d]Thread_usart run 10 hz...",count);
    //ros::spinOnce();
    loop_rate.sleep();
  }
  */
}
void Pthread_usart()
{
  pthread_t pid_usart;
  printf("topic thread start...\n");
  int pthread_spin = pthread_create(&pid_usart, NULL,Thread_usart, NULL);
}

int main(int argc, char **argv)
{
  printf("ros thread test start...\n");
  ros::init(argc, argv, "threadnode");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, CmdVel_CallBack);
  geometry_msgs::Twist msg;

  Pthread_usart();
  usleep(1000*1);
  Pthread_topic();
  
  printf("main thread start...\n");
  ros::Rate loop_rate(10);
  while (ros::ok()) 
  {
    // printf("main thread running...\n");
    //printf("IMU=%d,left_vel=%f,right_vel=%f\n\n",IMU_data,left_vel,right_vel);
    //ROS_INFO("IMU=[%d],left_vel=[%f],right_vel=[%f]", IMU_data,left_vel,right_vel);
    loop_rate.sleep();
    ros::spinOnce();
    
  }
  return 0;
}
