#include "camera_location.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

camera_location::camera_location()
:x_bias_(0),y_bias_(0),theta_bias_(0),bias_receive_complete_flag_(0),task_state_flag_(0),stop_flag_(0),begin_camera_navi_flag_(0)
{
	pub_vel_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	destination_task_state_ = n_.advertise<std_msgs::String>("/destination_task_state",1);
	sub_camera_pose_ = n_.subscribe("/camera_pose",2,&camera_location::cbCamera,this);
	sub_goal_task_ = n_.subscribe("/destination_pose", 1, &camera_location::cbGoalTask,this);

	pthreadRosSpin();

	while (ros::ok())
	{
		rosNavigation(x_goal_,y_goal_,oz_goal_,ow_goal_);
		ros::Duration(1).sleep();
	}
}

void camera_location::cbCamera(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	x_bias_ = msg->x;
	y_bias_ = msg->y;
	theta_bias_ = 2 * 3.14159265 - msg->theta;
	bias_receive_complete_flag_ = 1;
	cameraNavigation();
}

void camera_location::cbGoalTask(const geometry_msgs::Pose::ConstPtr& pose)
{
	if(task_state_flag_ == 0)
	{
		x_goal_ = pose->position.x;
		y_goal_ = pose->position.y;
		oz_goal_ = pose->orientation.z;
		ow_goal_ = pose->orientation.w;
		task_state_flag_ = 1;
	}
}

void camera_location::rosNavigation(double x,double y,double oz,double ow)
{
	if(task_state_flag_ == 0)
	{
		//cout << "no destination!" << endl;
		return;
	}

	MoveBaseClient ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("/map","/base_footprint",ros::Time(0),ros::Duration(3));
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        //ROS_INFO("Waiting for the move_base action server");
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = oz;
    goal.target_pose.pose.orientation.w = ow;

    //ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
    float x_tf = transform.getOrigin().x();
    float y_tf = transform.getOrigin().y();
    float oz_tf = transform.getRotation().getZ();
    float ow_tf = transform.getRotation().getW();
    while(1)
    {
        listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
        x_tf = transform.getOrigin().x();
        y_tf = transform.getOrigin().y();
        float distant2;
        distant2 = (x_tf - x) * (x_tf - x) + (y_tf - y) * (y_tf - y);
        if(distant2 <= 0.0009)
        {
            ac.cancelAllGoals();
            break;
        }

        if(ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {}
        else if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {}
        else
        {
			task_state_flag_ = 0;
            std_msgs::String state_of_goal;
            state_of_goal.data = "reach_the_destination's_nearby";
            destination_task_state_.publish(state_of_goal);
			return;
        }
    }

    begin_camera_navi_flag_ = 1;
}

void camera_location::cameraNavigation()
{
	if(begin_camera_navi_flag_ == 1)
	{
		if((x_bias_ < 0.001) && (y_bias_ < 0.001) && (theta_bias_ < 0.01))
		{
			stop_flag_++;
			stopAgv();
			return;
		}
		else
		{
			stop_flag_ = 0;
		}

		double angle_0;
		double det_angle;
		if (theta_bias_ > 3.14159265)
		{
			theta_bias_ = theta_bias_ - 2 * 3.14159265;
		}
		cout << x_bias_ << "," << y_bias_ << "," << theta_bias_ << endl;

		angle_0 = acos((0 - x_bias_) / sqrt(x_bias_ * x_bias_ + y_bias_ * y_bias_));
		if ((0 - y_bias_) < 0)
		{
			angle_0 = 0 - angle_0;
		}
		//cout << "car pose angle:" << theta_bias_ << endl;
		//cout << "       angel_0:" << angle_0 << endl;

		if (theta_bias_ - angle_0 < -3.14159265)
		{
			det_angle = theta_bias_ - angle_0 + 2 * 3.14159265;
			//cout << "the increment of angle11:" << det_angle * 180 / 3.14159265 << endl;
		}
		else if ((theta_bias_ - angle_0 >= -3.14159265) && (theta_bias_ - angle_0 <= 3.14159265))
		{
			det_angle = theta_bias_ - angle_0;
			//cout << "the increment of angle22:" << det_angle * 180 / 3.14159265 << endl;
		}
		else
		{
			det_angle = theta_bias_ - angle_0 - 2 * 3.14159265;
			//cout << "the increment of angle33:" << det_angle * 180 / 3.14159265 << endl;
		}

		double vel_x = 0.3;
		double vel_yal;
		if (det_angle < -3.14159265 / 2)
		{
			vel_x = 0 - vel_x;
			vel_yal = vel_x / (0.5 * sqrt(x_bias_ * x_bias_ + y_bias_ * y_bias_) / sin(det_angle + 3.14159265));
		}
		else if ((det_angle >= -3.14159265 / 2) && (det_angle < 0))
		{
			vel_yal = vel_x / (0.5 * sqrt(x_bias_ * x_bias_ + y_bias_ * y_bias_) / sin(0 - det_angle));
		}
		else if ((det_angle >= 0) && (det_angle < 3.14159265 / 2))
		{
			vel_yal = (0 - vel_x) / (0.5 * sqrt(x_bias_ * x_bias_ + y_bias_ * y_bias_) / sin(det_angle));
		}
		else
		{
			vel_x = 0 - vel_x;
			vel_yal = (0 - vel_x) / (0.5 * sqrt(x_bias_ * x_bias_ + y_bias_ * y_bias_) / sin(3.14159265 - det_angle));
		}

		//cout << "  vel_x:" << vel_x << endl << "vel_yal:" << vel_yal << endl;
		if(fabs(vel_yal) > 0.3)
		{
			vel_x = vel_x * 0.3 / fabs(vel_yal);
			vel_yal = vel_yal * 0.3 / fabs(vel_yal);
		}
		geometry_msgs::Twist pub_msg;
		pub_msg.linear.x = vel_x;
		pub_msg.angular.z = vel_yal;

		//cout << endl << "theta_bias:" << theta_bias_ << endl;
		if((x_bias_ * x_bias_ + y_bias_ * y_bias_ < 0.0001))
		{
			if(fabs(theta_bias_) < 0.1)
			{
				task_state_flag_ = 0;
		        begin_camera_navi_flag_ = 0;
				pub_msg.linear.x = 0;
				pub_msg.angular.z = 0;
				pub_vel_.publish(pub_msg);
				ros::Duration(0.2).sleep();
				return;
			}
			else if(theta_bias_ <= -0.1)
			{
				pub_msg.linear.x = 0;
				pub_msg.angular.z = 0.3;
				pub_vel_.publish(pub_msg);
			}
			else if(theta_bias_ >= 0.1)
			{
				pub_msg.linear.x = 0;
				pub_msg.angular.z = -0.3;
				pub_vel_.publish(pub_msg);
			}
		}
		else
	    {
		    pub_vel_.publish(pub_msg);
	    }
	}
}

void camera_location::stopAgv()
{
	cout << "stop the agv." << endl;
	if(stop_flag_ < 4)
	{
	    geometry_msgs::Twist pub_msg;
	    pub_msg.linear.x = 0;
	    pub_msg.angular.z = 0;
	    pub_vel_.publish(pub_msg);
	}
	else if(stop_flag_ == 5)
	{
		task_state_flag_ = 0;
		begin_camera_navi_flag_ = 0;
        std_msgs::String state_of_goal;
        state_of_goal.data = "reach_the_destination";
        destination_task_state_.publish(state_of_goal);
		stop_flag_ = 10;
	}
}

void camera_location::pthreadRosSpin()
{
    if(pthread_create(&m_tid_,NULL,threadRosSpin,(void*)this) != 0)
    {
        ROS_INFO("Start ros spin thread failed!");
        return; 
    }
}

void* camera_location::threadRosSpin(void* arg)
{
    camera_location *ptr =(camera_location*) arg;
    ptr->threadRosSpinRun();
    return NULL;
}

void camera_location::threadRosSpinRun()
{
	cout << "start ros spin thread!" << endl;
	ros::spin();
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"camera_location");
	camera_location agv_navigation;
	return 0;
}