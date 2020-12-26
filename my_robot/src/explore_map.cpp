#include <iostream>
#include <algorithm>
#include <cstdlib> 
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

double lidar_min_range;
std::vector<double> lidar_scans;
bool is_init = true;
bool robot_status = false;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explorer");
    ros::NodeHandle n;

    ros::Subscriber lidar_sub = n.subscribe("laser/scan", 1, lidarCallback);
    ros::ServiceServer service = n.advertiseService("set_robot_status", setStatus);

    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10.0);

    double dist_threshold = 3.0; //Distance threshold at which the robot would either turn away or stop
    bool is_random_turn = false; //boolean to indicate if the random turn is activated

    geometry_msgs::Twist twist_msg;

    srand((unsigned)time(0));

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

        double linear_vel, angular_vel;

        //finding obstacles in the way

        int no_of_scans = lidar_scans.size();
        if(no_of_scans == 0) //no lidar input, so skip processing
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
        
        if(front_dist > dist_threshold) //going straight
        {
            linear_vel = 0.5;
            angular_vel = 0.0;
        }

        else if(left_dist > dist_threshold) //turning left
        {
            linear_vel = 0.0;
            angular_vel = 0.3;
        }

        else if(right_dist > dist_threshold) //turning right
        {
            linear_vel = 0.0;
            angular_vel = -0.3;
        }

        else //all directions blocked
        {
            linear_vel = 0.0;
            angular_vel = 0.0;
        }

        //Random right turns to explore the map more
        int random_no = rand()%1500 + 1; //random no between 1 & 1500
        if(random_no >= 1490) //1 in 150 chance of a random turn
        {
            //reverse priority to left/right than above in order to explore new areas
            if(right_dist > dist_threshold)
            {
                linear_vel = 0.0;
                angular_vel = -0.5;
		    
		is_random_turn = true;
            }

            else if(left_dist > dist_threshold)
            {
                linear_vel = 0.0;
                angular_vel = 0.5;
		    
		is_random_turn = true;
            }
        }

        //std::cout<<"Cmd is lin: "<<linear_vel<<" and ang: "<<angular_vel<<std::endl;

        //publishing twist command
        twist_msg.linear.x = linear_vel;
        twist_msg.angular.z = angular_vel;
        twist_pub.publish(twist_msg);
	    
	if(is_random_turn)
        {
            std::cout<<"----Random turn-----"<<std::endl;
            ros::Duration(0.4).sleep(); //keep turning for 0.4 sec
            is_random_turn = false;
        }

        loop_rate.sleep();
    }

    return 0;
}
