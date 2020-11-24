#include <iostream>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double heading=0.0, curr_x=0.0, curr_y=0.0, lidar_min_range;
std::vector<double> lidar_scans;
bool is_init = true;
bool robot_status = false;

double getYaw(geometry_msgs::Quaternion q_msg)
{
   tf2::Quaternion quat;
   tf2::fromMsg(q_msg, quat);

   tf2::Matrix3x3 mat(quat);
   double roll, pitch, yaw;
   mat.getRPY(roll, pitch, yaw);

   return yaw;
}

bool setStatus(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    res.success = true;
    robot_status = req.data;
    if(robot_status)
    {
        res.message = "Robot Enabled";
    }
    else
    {
        res.message = "Robot Disabled";
    }

    return true;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//std::cout<<"In Lidar callback"<<std::endl;
    
    int no_of_scans = ((msg->angle_max - msg->angle_min)/msg->angle_increment) + 1;

    lidar_min_range = msg->range_min;

    if(is_init)
    {
        for(int i=0;i<no_of_scans;i++)
            lidar_scans.push_back(0.0);

        is_init = false;
    }

    for(int i=0;i<no_of_scans;i++)
    {
        lidar_scans[i] = msg->ranges[i];
    }

    return;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    curr_x = msg->pose.pose.position.x;
    curr_y = msg->pose.pose.position.y;
    heading = getYaw(msg->pose.pose.orientation);

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigator");
    ros::NodeHandle n, private_nh("~");

    ros::Subscriber lidar_sub = n.subscribe("laser/scan", 1, lidarCallback);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);
    ros::ServiceServer service = n.advertiseService("set_robot_status", setStatus);

    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10.0);

    double dist_threshold; //Distance threshold at which the robot would either turn away or stop
    double goal_threshold; //If goal is within this distance, it is assumed reached
    double heading_tolerance;
    double goal_x, goal_y; //goal position in odom frame

    private_nh.param("dist_threshold", dist_threshold, 3.0);
    ROS_INFO("dist_threshold: %f", dist_threshold);
    private_nh.param("goal_threshold", goal_threshold, 0.8);
    ROS_INFO("goal_threshold: %f", goal_threshold);
    private_nh.param("heading_tolerance", heading_tolerance, 0.052);
    ROS_INFO("heading_tolerance: %f", heading_tolerance);
    private_nh.param("goal_x", goal_x, 20.0);
    ROS_INFO("goal_x: %f", goal_x);
    private_nh.param("goal_y", goal_y, 5.0);
    ROS_INFO("goal_y: %f", goal_y);

    geometry_msgs::Twist twist_msg;
    double linear_vel, angular_vel, init_diff_heading;
    bool is_first_turn = true;

    while(ros::ok())
    {
        ros::spinOnce();

        if(!robot_status) //robot is disabled, so skip processing
        {
            //publishing twist command
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            twist_pub.publish(twist_msg);

            loop_rate.sleep();
            continue;
        }

        //finding obstacles in the way

        int no_of_scans = lidar_scans.size();
        if(no_of_scans == 0) //no lidar input
        {
            loop_rate.sleep();
            continue;
        }

        //Dividing Lidar FoV int 3 equal parts, i.e., Right, Front, Left

        //-------------Right--------------------
        double right_dist = *std::min_element(lidar_scans.begin(), lidar_scans.begin()+(no_of_scans/3));
        if(right_dist < lidar_min_range)
            right_dist = 100; //some large value
        
        //-------------Front--------------------
        double front_dist = *std::min_element(lidar_scans.begin()+(no_of_scans/3), lidar_scans.begin()+(2*no_of_scans/3));
        if(front_dist < lidar_min_range)
            front_dist = 100; //some large value
        
        //-------------Left--------------------
        double left_dist = *std::min_element(lidar_scans.begin()+(2*no_of_scans/3), lidar_scans.end());
        if(left_dist < lidar_min_range)
            left_dist = 100; //some large value

        //std::cout<<"Front: "<<front_dist<<" ,Left: "<<left_dist<<" ,Right: "<<right_dist<<std::endl;

        double dist_to_goal = sqrt((pow((goal_x - curr_x),2)) + (pow((goal_y - curr_y),2)));
        double target_heading = atan2((goal_y - curr_y), (goal_x - curr_x));
        double diff_heading = target_heading - heading;
        if(is_first_turn)
        {
            init_diff_heading =  diff_heading;

            is_first_turn = false;
        }

        std::cout<<"Heading difference: "<<diff_heading*(180/3.1417)<<std::endl; 

        if(dist_to_goal <= goal_threshold) //reached goal
        {
            linear_vel = 0.0;
            angular_vel = 0.0;

            std::cout<<"----------------Reached the Goal-----------------"<<std::endl;
        }

        else if(fabs(diff_heading) > heading_tolerance) //turning towards goal
        {
            //std::cout<<"Heading adjustment"<<std::endl;

            if(diff_heading > 0 && left_dist > dist_threshold) //turning left
            {
                std::cout<<"Left adjustment"<<std::endl;
                linear_vel = 0.0;
                angular_vel = 0.3;
            }

            else if(diff_heading < 0 && right_dist > dist_threshold) //turning right
            {
                std::cout<<"Right adjustment"<<std::endl;
                linear_vel = 0.0;
                angular_vel = -0.3;
            }

            else if(front_dist > dist_threshold)
            {
                linear_vel = 0.5;
                angular_vel = 0.0;
            }
        }

        else
        {
            std::cout<<"Avoiding objects"<<std::endl;
            
            //continue with obstacle avoidance and explorer logic
            if(front_dist > dist_threshold) //going straight
            {
                linear_vel = 0.5;
                angular_vel = 0.0;
            }

            //Turning left/right first according to initial goal heading
            else if(init_diff_heading >= 0)
            {
                if(left_dist > dist_threshold) //turning left
                {
                    linear_vel = 0.0;
                    angular_vel = 0.3;
                }

                else if(right_dist > dist_threshold) //turning right
                {
                    linear_vel = 0.0;
                    angular_vel = -0.3;
                }
            }

            //Turning left/right first according to initial goal heading
            else if(init_diff_heading < 0)
            {
                if(right_dist > dist_threshold) //turning right
                {
                    linear_vel = 0.0;
                    angular_vel = -0.3;
                }

                else if(left_dist > dist_threshold) //turning left
                {
                    linear_vel = 0.0;
                    angular_vel = 0.3;
                }
            }

            else //all directions blocked
            {
                linear_vel = 0.0;
                angular_vel = 0.0;
            }
        }
        
        //std::cout<<"Cmd is lin: "<<linear_vel<<" and ang: "<<angular_vel<<std::endl;

        //publishing twist command
        twist_msg.linear.x = linear_vel;
        twist_msg.angular.z = angular_vel;
        twist_pub.publish(twist_msg);

        loop_rate.sleep();
    }

    return 0;
}