//
// Created by larr-lyw on 21. 8. 11..
//
#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_traj_airsim_node");
    ros::NodeHandle nh("~");


    string vehicle_name;
    nh.param<string>("vehicle_name",vehicle_name,"");

    ros::Publisher airsim_local_goal_pub;
    airsim_local_goal_pub = nh.advertise<nav_msgs::Odometry>("/airsim_node/"+vehicle_name+"/local_goal_enu",1);

    nav_msgs::Odometry local_goal_info;

    double goal_x;
    double goal_y;
    double goal_z;
    double goal_yaw;

    ros::Rate loop_rate(100);
    double time_init = ros::Time::now().toSec();

    double time_now = ros::Time::now().toSec();
    while(ros::ok())
    {
        time_now = ros::Time::now().toSec();
        if(vehicle_name=="Drone1")
        {
            local_goal_info.pose.pose.position.x = 0.0 - 0.0*(time_now-time_init);
            local_goal_info.pose.pose.position.y = 0.0 -0.2*(time_now-time_init) - sin(0.05*(time_now-time_init));
            local_goal_info.pose.pose.position.z =  0.05*(time_now-time_init);
            local_goal_info.pose.pose.orientation.w = 1.0;
        }
        else
        {
            local_goal_info.pose.pose.position.x = 0.05*(time_now-time_init);
            local_goal_info.pose.pose.position.y = -0.5*(time_now-time_init);
            local_goal_info.pose.pose.position.z = - 0.1*(time_now-time_init);
            local_goal_info.pose.pose.orientation.w = 1.0;
        }



        airsim_local_goal_pub.publish(local_goal_info);
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;

}



//
//13 int main(int argc, char ** argv)
//14 {
//    15     ros::init(argc, argv, "try_example_test");
//    16     ROS_INFO("ROS START");
//    17     // Ready for file reading
//    18     // ROS PARAM INITIALIZE
//    19     RosWrapper YW_ros_wrapper;
//    20     YW_ros_wrapper.run();
//    21
//    22
