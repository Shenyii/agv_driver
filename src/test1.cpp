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

void getBias()
{
    double x0,y0,ow0,oz0;
    double x1,y1,ow1,oz1;
    std::cout << "please input current position." << std::endl;
    std::cin >> x0 >> y0 >> ow0 >> oz0;
    std::cout << "please input expect position." << std::endl;
    std::cin >> x1 >> y1 >> ow1 >> oz1;

    tf::Quaternion agv_in_map(0,0,oz0,ow0);
    tf::Matrix3x3 matrix;
    matrix.setRotation(agv_in_map);
    double distance_bias_ = matrix[0][1] * (x1 - x0) + matrix[1][1] * (y1 - y0);

    double sin_angle_bias;
    double cos_angle_bias;
    double distance_bias;

    distance_bias = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    cos_angle_bias = (matrix[0][0] * (x1 - x0) + matrix[1][0] * (y1 - y0)) / distance_bias;
    sin_angle_bias = distance_bias_ / distance_bias;
    double angle_bias_ = acos(cos_angle_bias);
    if(sin_angle_bias < 0)
    {
        angle_bias_ = 0 - angle_bias_;
    }
    if(matrix[2][2] < 0)
    {
        angle_bias_ = 0 - angle_bias_;
    }
    ROS_INFO("current:%f,%f,%f,%f.",x0,y0,ow0,oz0);
    ROS_INFO("hope   :%f,%f,%f,%f.",x1,y1,ow1,oz1);
    ROS_INFO("angle_bias:%f",angle_bias_);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_angle_bias");
    ros::NodeHandle n;
    while(ros::ok())
    {
        getBias();
        ROS_INFO("hellow!!");
    }
}
