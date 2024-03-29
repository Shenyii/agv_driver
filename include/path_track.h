#pragma once
#include <vector>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include "tf_listerner.h"
#include <pthread.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

class pose
{
public:
	double x;
	double y;
	double ow;
	double oz;

	pose()
	:x(0.0),y(0.0),ow(1.0),oz(0.0)
	{};

	pose(double x1,double x2,double x3,double x4)
	:x(x1),y(x2),ow(x3),oz(x4)
	{};
};

class path_track
{
public:
	path_track();
	~path_track();
	void pidControl();

private:
	vector<pose> path_;
	geometry_msgs::Pose destination_;
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher pub_arrived_destination_;
	ros::Subscriber sub_;
	ros::Subscriber sub_destination_;
	
	geometry_msgs::Twist cmd_vel_;
	double angle_velocity_;

	double angle_bias_;
	double distance_bias_;
	double pid_p_;
	double pid_i_;
	double pid_d_;
	double ave_vel_;
	double integral_part_;
	double last_bias_;
	double calculated_frequency_;
	int complete_path_track_flag_;

	Tf_Listerner* current_position_;
	
	void callBackPath(const nav_msgs::Path::ConstPtr& path_point);
	void callBackDestination(const geometry_msgs::Pose::ConstPtr& destination_pose);
	void getBias();
	void motionConstraint();
	void sendVelocityCommand();
	void goToDestination();
	double goToDestinationAngle();

	void pthreadAutoNavigation();
    static void *threadAutoNavigation(void * arg);
    void threadRun();
    pthread_t m_tid_;
};

