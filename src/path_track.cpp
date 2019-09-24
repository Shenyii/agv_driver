#include "path_track.h"

path_track::path_track()
:angle_bias_(0),distance_bias_(0),complete_path_track_flag_(2),calculated_frequency_(20.0)
{
    current_position_ = new Tf_Listerner("map","base_footprint");
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",2);
    pub_arrived_destination_ = n_.advertise<std_msgs::String>("whether_arrived_destination",1);
    sub_ = n_.subscribe("/move_base/NavfnROS/plan",2,&path_track::callBackPath,this);
    sub_destination_ = n_.subscribe("/destination_pose",2,&path_track::callBackDestination,this);
    ros::Duration(3).sleep();
    cout << "begin the path track program." << endl;

    n_.param<double>("pid_p",pid_p_,3.0);
    n_.param<double>("pid_i",pid_i_,1.0);
    n_.param<double>("pid_d",pid_d_,1.0);
    n_.param<double>("ave_vel",ave_vel_,0.3);
    n_.getParam("path_track/pid_p",pid_p_);
    n_.getParam("path_track/pid_i",pid_i_);
    n_.getParam("path_track/pid_d",pid_d_);
    n_.getParam("path_track/ave_vel",ave_vel_);

    pthreadAutoNavigation();

    while(ros::ok())
    {
        ros::spinOnce();
        if(complete_path_track_flag_ == 0 || complete_path_track_flag_ == 1)
        {
            sendVelocityCommand();
        }
        ros::Duration(1 / calculated_frequency_).sleep();
    }
}

path_track::~path_track()
{}

void path_track::pidControl()
{
    getBias();
    //ROS_INFO("angle_bias is:%f",angle_bias_);
    integral_part_ = integral_part_ + angle_bias_ / calculated_frequency_;
    angle_velocity_ = pid_p_ * angle_bias_ +pid_p_ * integral_part_ + pid_d_ * (angle_bias_ - last_bias_) * calculated_frequency_;
    
    last_bias_ = angle_bias_;
}

void path_track::callBackPath(const nav_msgs::Path::ConstPtr& path_point)
{
    integral_part_ = 0;
    last_bias_ = 0;

    pose position;
    path_.clear();
    for(int i = 0;i < path_point->poses.size();i++)
    {
        position.x = path_point->poses[i].pose.position.x;
        position.y = path_point->poses[i].pose.position.y;
        if(i < path_point->poses.size() - 1)
        {
            float x;
            float y;
            float ow;
            float oz;
            x = path_point->poses[i + 1].pose.position.x - position.x;
            y = path_point->poses[i + 1].pose.position.y - position.y;
            ow = cos(acos(x / sqrt(x * x + y * y)) / 2);
            oz = sqrt(1 - ow * ow);
            if(y < 0) oz = 0 - oz;
            position.ow = ow;
            position.oz = oz;

            path_.push_back(position);
        }
        else
        {
            float x1 = path_point->poses[i].pose.position.x - destination_.position.x;
            float y1 = path_point->poses[i].pose.position.y - destination_.position.y;
            float ow1 = cos(acos(x1 / sqrt(x1 * x1 + y1 * y1)) / 2);
            float oz1 = sqrt(1 - ow1 * ow1);
            if(y1 < 0) oz1 = 0 - oz1;
            position.ow = ow1;
            position.oz = oz1;

            if((x1 * x1 + y1 * y1) > 0.01)
            {
                path_.push_back(position);
                position.x = destination_.position.x;
                position.y = destination_.position.y;
                position.ow = destination_.orientation.w;
                position.oz = destination_.orientation.z;
                path_.push_back(position);
            }
            else
            {
                position.x = destination_.position.x;
                position.y = destination_.position.y;
                position.ow = destination_.orientation.w;
                position.oz = destination_.orientation.z;
                path_.push_back(position);
            }
        }
    }
}

void path_track::callBackDestination(const geometry_msgs::Pose::ConstPtr& destination_pose)
{
    if(complete_path_track_flag_ != 0)
    {
        destination_.position.x = destination_pose->position.x;
        destination_.position.y = destination_pose->position.y;
        destination_.position.z = destination_pose->position.z;
        destination_.orientation.x = destination_pose->orientation.x;
        destination_.orientation.y = destination_pose->orientation.y;
        destination_.orientation.z = destination_pose->orientation.z;
        destination_.orientation.w = destination_pose->orientation.w;
        complete_path_track_flag_ = 0;
    }
}

void path_track::getBias()
{
    float bias;
    int n = 0;
    float min_distance = 1.0;
    float x;
    float y;
    float oz;
    float ow;
    x = current_position_->x();
    y = current_position_->y();
    oz = current_position_->oz();
    ow = current_position_->ow();
    if(ow < 0)
    {
        ow = 0 - ow;
        oz = 0 - oz;
    }

    for(int i = 0;i < path_.size() - 1;i++)
    {
        float distance;
        distance = sqrt((x-path_[i].x)*(x-path_[i].x)+(y-path_[i].y)*(y-path_[i].y));

        if(distance < min_distance)
        {
            min_distance = distance;
            n = i;
        }
    }
    if(n > path_.size() - 2)
    {
        n = path_.size() - 2;
    }

    if(path_.size() == 1)
    {
        n = -1;
    }

    tf::Quaternion agv_in_map(0,0,oz,ow);
    tf::Matrix3x3 matrix;
    matrix.setRotation(agv_in_map);
    distance_bias_ = matrix[0][1] * (path_[n + 1].x - x) + matrix[1][1] * (path_[n + 1].y - y);

    double sin_angle_bias;
    double cos_angle_bias;
    double distance_bias;

    distance_bias = sqrt((path_[n + 1].x - x) * (path_[n + 1].x - x) + (path_[n + 1].y - y) * (path_[n + 1].y - y));
    cos_angle_bias = (matrix[0][0] * (path_[n + 1].x - x) + matrix[1][0] * (path_[n + 1].y - y)) / distance_bias;
    sin_angle_bias = distance_bias_ / distance_bias;
    angle_bias_ = acos(cos_angle_bias);
    if(sin_angle_bias < 0)
    {
        angle_bias_ = 0 - angle_bias_;
    }
    if(matrix[2][2] < 0)
    {
        angle_bias_ = 0 - angle_bias_;
    }
    ROS_INFO("current:%f,%f,%f,%f.",x,y,ow,oz);
    ROS_INFO("hope   :%f,%f,%f,%f.",path_[n+1].x,path_[n+1].y,path_[n+1].ow,path_[n+1].oz);
    ROS_INFO("angle_bias:%f",angle_bias_);

    if(n > 0)
    {
        path_.erase(path_.begin() + 1,path_.begin() + n + 1);
    }
}

void path_track::motionConstraint()
{
    cmd_vel_.linear.x = ave_vel_;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    //cmd_vel_.angular.z = 0.0;
    if(fabs(angle_velocity_) > ave_vel_)
    {
        cmd_vel_.linear.x = cmd_vel_.linear.x * ave_vel_ / fabs(angle_velocity_);
        cmd_vel_.angular.z = angle_velocity_ / fabs(angle_velocity_) * ave_vel_;
    }
    //pub_.publish(cmd_vel_);
}
void path_track::sendVelocityCommand()
{
    if(path_.size() == 0)
    {
        return;
    }
    else if(path_.size() > 2)
    {
        if(complete_path_track_flag_ == 0)
        {
            pidControl();
            motionConstraint();
            pub_.publish(cmd_vel_);
        }
        else
        {
            cmd_vel_.linear.x = 0;
            cmd_vel_.linear.y = 0;
            cmd_vel_.linear.z = 0;
            cmd_vel_.angular.x = 0.0;
            cmd_vel_.angular.y = 0.0;
            cmd_vel_.angular.z = 0.0;
            pub_.publish(cmd_vel_);
            complete_path_track_flag_ = 2;
            std_msgs::String state_of_task;
            state_of_task.data = "arrived_destination";
            pub_arrived_destination_.publish(state_of_task);
        }
    }
    else if(complete_path_track_flag_ == 0)
    {
        pidControl();
        goToDestination();
        pub_.publish(cmd_vel_);
    }
    else if(complete_path_track_flag_ == 1)
    {
        for(int i = 0;i < 5;i++)
        {
            cmd_vel_.linear.x = 0;
            cmd_vel_.linear.y = 0;
            cmd_vel_.linear.z = 0;
            cmd_vel_.angular.x = 0.0;
            cmd_vel_.angular.y = 0.0;
            cmd_vel_.angular.z = 0.0;
            pub_.publish(cmd_vel_);
        }
        complete_path_track_flag_ = 2;
        std_msgs::String state_of_task;
        state_of_task.data = "arrived_destination";
        pub_arrived_destination_.publish(state_of_task);
    }
}

void path_track::goToDestination()
{
    double det_x = current_position_->x() - destination_.position.x;
    double det_y = current_position_->y() - destination_.position.y;
    if(det_x * det_x + det_y * det_y > 0.3)
    {
        motionConstraint();
    }
    else if(fabs(goToDestinationAngle()) > 1)
    {
        cmd_vel_.linear.x = 0;
        cmd_vel_.linear.y = 0;
        cmd_vel_.linear.z = 0;
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z = ave_vel_;
    }
    else
    {
        complete_path_track_flag_ = 1;
        path_.clear();
    }
}

double path_track::goToDestinationAngle()
{
    double angle_of_agv;
    double angle_of_destination;
    angle_of_agv = 2 * acos(current_position_->ow());
    if(current_position_->oz() < 0)
    {
        angle_of_agv = 0 - angle_of_agv;
    }
    angle_of_destination = 2 * acos(destination_.orientation.w);
    if(destination_.orientation.z < 0)
    {
        angle_of_destination = 0 - angle_of_destination;
    }
    return angle_of_agv - angle_of_destination;
}

void path_track::pthreadAutoNavigation()
{
    if(pthread_create(&m_tid_,NULL,threadAutoNavigation,(void*)this) != 0)
    {
        std::cout << "Start tf listerner thread failed!" << std::endl;
        return; 
    }
}

void* path_track::threadAutoNavigation(void * arg)
{
    path_track *ptr =(path_track*) arg;
    ptr->threadRun();
    return NULL;
}

void path_track::threadRun()
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    while(ros::ok())
    {
        if(complete_path_track_flag_ == 0)
        {
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = destination_.position.x;
            goal.target_pose.pose.position.y = destination_.position.y;
            goal.target_pose.pose.orientation.z = destination_.orientation.z;
            goal.target_pose.pose.orientation.w = destination_.orientation.w;

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            while(ros::ok())
            {
                double det_x = current_position_->x() - destination_.position.x;
                double det_y = current_position_->y() - destination_.position.y;
                double distance = det_x * det_x + det_y * det_y;
                if(distance <= 0.4)
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
                    complete_path_track_flag_ = 1;
                    std_msgs::String state_of_goal;
                    state_of_goal.data = "reach_the_destination's_nearby";
                    pub_arrived_destination_.publish(state_of_goal);
                    for(int i = 0;i < 5;i++)
                    {
                        cmd_vel_.linear.x = 0;
                        cmd_vel_.linear.y = 0;
                        cmd_vel_.linear.z = 0;
                        cmd_vel_.angular.x = 0.0;
                        cmd_vel_.angular.y = 0.0;
                        cmd_vel_.angular.z = 0.0;
                        pub_.publish(cmd_vel_);
                    }
                    break;
                }
            }
        }
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "path_track");
    
    path_track path_track_test;

    return 0;
}
