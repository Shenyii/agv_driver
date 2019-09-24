#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

class lidar_data_handle
{
public:
    lidar_data_handle();
    ~lidar_data_handle();
    bool whether_there_are_obstacle();

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_velcity_;
    ros::Publisher pub_velcity_;
    int obstacle_flag_;
    
    void point_cb(const sensor_msgs::LaserScan::ConstPtr& in_cloud);
    void velcity_cb(const geometry_msgs::Twist::ConstPtr& velcity);
};