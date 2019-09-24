#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
