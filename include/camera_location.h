#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <std_msgs/String.h>
#include <pthread.h>
#include <signal.h>

using namespace std;

class camera_location
{
public:
    camera_location();
    void cbCamera(const geometry_msgs::Pose2D::ConstPtr& msg);
    void cbGoalTask(const geometry_msgs::Pose::ConstPtr& pose);
    void rosNavigation(double x,double y,double oz,double ow);
    void cameraNavigation();

private:
    ros::Publisher pub_vel_;
    ros::Publisher destination_task_state_;
    ros::Subscriber sub_camera_pose_;
    ros::Subscriber sub_goal_task_;
    ros::NodeHandle n_;
    pthread_t m_tid_;
    void pthreadRosSpin();
    static void* threadRosSpin(void* arg);
    void threadRosSpinRun();

    double x_bias_;
    double y_bias_;
    double theta_bias_;
    int bias_receive_complete_flag_;
    int task_state_flag_;
    int begin_camera_navi_flag_;
    double x_goal_;
    double y_goal_;
    double oz_goal_;
    double ow_goal_;

    int stop_flag_;
    void stopAgv();
};