//Commander ROS client, Assignment 2
//Akhil Kandhari
//axk751@case.edu
//Client sends out a frequency and amplitude for wave generation to a service which gives feedback when inputs have been received.

#include <ros/ros.h>
#include <commander_ros_service/CommanderServiceMsg.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_ros_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<commander_ros_service::CommanderServiceMsg>("wave_inputs");
    commander_ros_service::CommanderServiceMsg srv;

    long int g_freq;
    long int g_amp;

    while(ros::ok()) {
        cout<<endl;
        cout<<"Enter frequency for wave: ";
        cin>>g_freq;
        cout<<endl;
        cout<<"Enter amplitude for wave: ";
        cin>>g_amp;

        srv.request.frequency = g_freq;
        srv.request.amplitude = g_amp;

        if (client.call(srv)) {
            cout << srv.response.receipt << endl;
        }

    }
    return 0;
}