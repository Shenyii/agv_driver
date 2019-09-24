#include "lidar_obstacle_check.h"
#include <iostream>

using namespace std;

lidar_data_handle::lidar_data_handle()
: obstacle_flag_(0)
{
    sub_point_cloud_ = n_.subscribe("/scan",5,&lidar_data_handle::point_cb,this);
    sub_velcity_ = n_.subscribe("/cmd_vel",5,&lidar_data_handle::velcity_cb,this);
    pub_velcity_ = n_.advertise<geometry_msgs::Twist>("vel_to_stm32",5);
    ROS_INFO("Checking the obstacle.");
    while(ros::ok())
    {
        ros::spinOnce();
    }
}

lidar_data_handle::~lidar_data_handle()
{}

bool lidar_data_handle::whether_there_are_obstacle()
{
    return obstacle_flag_;
}

void lidar_data_handle::point_cb(const sensor_msgs::LaserScan::ConstPtr& in_cloud)
{
    int obstacle_point = 0;
    for(int i = 0;i < 213;i++)
    {
        if(in_cloud->ranges[i] < 0.33)
        {
            if(in_cloud->ranges[i] * cos(i * 3.14159265 / 1000.0) < 0.25)
            {
                obstacle_point++;
            }
        }
    }
    for(int i = 213;i < 787;i++)
    {
        if(in_cloud->ranges[i] < 0.33)
        {
            if(fabs(in_cloud->ranges[i] * sin(i * 3.14159265 / 1000.0)) < 0.2)
            {
                obstacle_point++;
            }
        }
    }
    for(int i = 787;i < 1000;i++)
    {
        if(in_cloud->ranges[i] < 0.33)
        {
            if(fabs(in_cloud->ranges[i] * cos(i * 3.14159265 / 1000.0)) < 0.25)
            {
                obstacle_point++;
            }
        }
    }

    if(obstacle_point > 10) obstacle_flag_ = 1;
    else obstacle_flag_ = 0;
}

void lidar_data_handle::velcity_cb(const geometry_msgs::Twist::ConstPtr& velcity)
{
    geometry_msgs::Twist vel;
    if(whether_there_are_obstacle())
    {
        vel.linear.x = 0;
        vel.angular.z = 0;
        pub_velcity_.publish(vel);
    }
    else
    {
        vel.linear.x = velcity->linear.x;
        vel.angular.z = velcity->angular.z;
        pub_velcity_.publish(vel);
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_obstacle_check");
    lidar_data_handle lidar_data_handle1;
}

