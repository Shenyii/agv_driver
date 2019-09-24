#include "odometry.h"
#include "uart_comm.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

const    int speed_ratio = 4096;      //4096*4;
const double wheel_perimeter = 53.4;   //cm
const double bias = 30;               //cm
const double frequency_encode = 10;   //ignore it, useless
int  debugFlag = 0;
bool use_imu_heading = true;
Opose poseUpdate;
Opose poseUpdate_encode;
double dtime = 0.0;

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
  //move_angle_encode = wheel_perimeter*(encodeR - encodeL)/speed_ratio/bias;
  move_angle_encode = 85*(encodeR - encodeL)/speed_ratio/38;
  if(use_imu_heading == true)
  {
    move_angle = IMU* 0.0174532925;  //3.1415926/180;
    //move_angle = move_angle_encode;
  }

  //printf("dis=%f,IMU=%d.\n",move_dis,IMU);
  PoseTranslation(move_dis,move_angle,poseForward);  
  geometry_msgs::Twist movetwist;
 
  movetwist.linear.x = poseForward.x/dtime;//frequency_encode;
  movetwist.angular.z = move_angle_encode/dtime;//frequency_encode;
  Pose_Update(poseForward);
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(std::atan2(poseForward.heading[2],poseForward.heading[0]));
  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(std::atan2(poseUpdate.heading[2],poseUpdate.heading[0]));
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
