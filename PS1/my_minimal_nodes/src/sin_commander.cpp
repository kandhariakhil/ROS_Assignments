/* Commander for controller that prompts amplitude and frewuency and commands sinusoidal velocities to the minimal controller
    Akhil Kandhari
    axk751@case.edu
    Ver 1.0 9/18/2018

    The concept of this code will be to subscribe to a user input value for amplitude and frequency and publish the velocity continuously at
    a given rate to the controller which then is connected to the simulator to change the velocity using the force_cmd

    This code is going to subscribe to a user input vel_cmd and publish that to minimal_controller
*/

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<math.h>

std_msgs::Float64 g_amp;
std_msgs::Float64 g_freq;
std_msgs::Float64 g_vel_cmd;

//Uncomment below if subscriber available
/*
void myCallbackAmp(const std_msgs::Float64& message_holder){
    //Checks for messages on topic wave_info_amp
    ROS_INFO("Received wave Amplitude is: %f", message_holder.data);
    g_amp.data = message_holder.data;
}

void myCallbackFreq(const std_msgs::Float64& message_holder){
    //Checks for messages on topic wave_info_freq
    ROS_INFO("Received wave frequency is: %f", message_holder.data);
    g_freq.data = message_holder.data;
}
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander"); //When this is compiled and run, ROS will recognize this node as sin_commander
    ros::NodeHandle nh; //Node handle created

    //Comment code below if subscriber available
    ///*
    std::cout<<"Enter wave amplitude "; //Remove this line if it does not work
    std::cin>>g_amp.data;
    ROS_INFO("Received wave Amplitude is: %f", g_amp.data);


    std::cout<<"Enter wave frequency "; //Remove this line if it does not work
    std::cin>>g_freq.data;
    ROS_INFO("Received wave Amplitude is: %f", g_freq.data);
    //*/

    //Uncomment below is subscriber available
    /*
    //Create subscriber that will wait for user input based on a callback function
    ros::Subscriber user_input_object1 = nh.subscribe("wave_info_amp",1,myCallbackAmp);
    ros::Subscriber user_input_object2 = nh.subscribe("wave_info_freq",1,myCallbackFreq);
    */

    //Create publisher that will publish the user input data to minimal_controller
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
        g_vel_cmd.data = g_amp.data * sin(2 * 3.14 * g_freq.data * dt_commander);
        my_publishing_object.publish(g_vel_cmd); //Publish the command as g_vel_cmd
        ROS_INFO("velocity command = %f", g_vel_cmd.data);
        ros::spinOnce();
        naptime.sleep();
        dt_commander = dt_commander+sample_rate;
    }
    return 0; //should never get here, unless roscore dies
}

