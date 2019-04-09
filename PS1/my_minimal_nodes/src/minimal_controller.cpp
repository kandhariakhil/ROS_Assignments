/*  Minimal controller node:
    Akhil Kandhari
    Written to test for controller
    Subscribes to two topics velocity and vel_cmd, and publish to topic force_cmd
    at each control cycle, the controller checks for the latest system state, checks for updates to commanded vel. and computes a proportional feedback error to derive and publisj a force command
*/

#include<ros/ros.h>
#include<std_msgs/Float64.h>

std_msgs::Float64 g_velocity;
std_msgs::Float64 g_force;
std_msgs::Float64 g_vel_cmd;

void myCallbackVelocity(const std_msgs::Float64& message_holder){
    //Checks got messages on topic velocity
    ROS_INFO("Received velocity is: %f", message_holder.data);
    g_velocity.data = message_holder.data;
}

void myCallbackVelCmd(const std_msgs::Float64& message_holder){
    //Checks got messages on topic vel_cmd
    ROS_INFO("Received velocity command is: %f", message_holder.data);
    g_vel_cmd.data = message_holder.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimal_controller");
    //When this compiled node is run, ROS will recognize this as a node called minimal_controller
    ros::NodeHandle nh; //Node handle created
    //Create first subscriber and have it subscribe to the topic velocity
    ros::Subscriber my_subscriber_object1 = nh.subscribe("velocity",1, myCallbackVelocity);
    //Create second subscriber and have it subscribe to the topic vel_cmd
    ros::Subscriber my_subscriber_object2 = nh.subscribe("vel_cmd",1, myCallbackVelCmd);
    //Create publisher that takes into account the velocities and publishes a force command
    ros::Publisher my_publishing_object = nh.advertise<std_msgs::Float64>("force_cmd", 1);

    //Development of actual controller

    double Kv = 1.0; //Velocity feedback gain
    double dt_controller = 0.1; //Specify 10 Hz controller sample rate
    double sample_rate = 1.0/dt_controller; //Compute the corresponding update frequency
    ros::Rate naptime(sample_rate); //Used to regulate the loop
    g_velocity.data = 0.0;
    g_force.data = 0.0;
    g_vel_cmd.data = 0.0;

    double vel_err = 0.0; //Error of velocity commanded and true velocity

    while(ros::ok())
    {
        vel_err = g_vel_cmd.data - g_velocity.data; //computes error
        g_force.data = Kv*vel_err;
        my_publishing_object.publish(g_force); //publish the control effort computed by this controller

        ROS_INFO("force command = %f", g_force.data);
        ros::spinOnce();
        naptime.sleep();
    }
    return 0; //should never get here, unless roscore dies
}
