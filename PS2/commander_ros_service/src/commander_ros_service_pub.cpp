#include <ros/ros.h>
#include <commander_ros_service/CommanderServiceMsg.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include<math.h>
using namespace std;

std_msgs::Float64 g_vel_cmd;
std_msgs::Float64 g_freq;
std_msgs::Float64 g_amp;
std_msgs::Float64 input_freq;
std_msgs::Float64 input_amp;

bool callback(commander_ros_service::CommanderServiceMsgRequest& request, commander_ros_service::CommanderServiceMsgResponse& response)
{
    ROS_INFO("callback activated");

    long int g_freq(request.frequency);
    long int g_amp(request.amplitude);

    input_freq.data = g_freq;
    input_amp.data = g_amp;

    ROS_INFO("Amplitude and frequency received for wave generation, forwarding received values to sin_commander");

    response.receipt = "Wave being generated in service terminal" ;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_ros_service");
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::ServiceServer service = n.advertiseService("wave_inputs", callback);

    ROS_INFO("Ready to receive wave frequency and amplitude");

    ros::Publisher my_publishing_object = nh.advertise<std_msgs::Float64>("vel_cmd",1);

    //Development of actual commander

    double dt_commander = 0.1; //Specify 10 HZ as sample rate
    double sample_rate = 1.0/dt_commander;
    ros::Rate naptime(sample_rate);

    //g_amp.data = 0.0;
    //g_freq.data = 0.0;
    g_vel_cmd.data = 0.0;

    while(ros::ok())
    {  
        g_vel_cmd.data = input_amp.data * sin(2 * 3.14 * input_freq.data * dt_commander);
        my_publishing_object.publish(g_vel_cmd); //Publish the command as g_vel_cmd
        ROS_INFO("velocity command = %f", g_vel_cmd.data);
        ros::spinOnce();
        naptime.sleep();
        dt_commander = dt_commander+sample_rate;
    }
    return 0; //should never get here, unless roscore dies
}