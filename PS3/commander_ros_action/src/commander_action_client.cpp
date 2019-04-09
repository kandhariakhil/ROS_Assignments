#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>

using namespace std;

#include<commander_ros_action/commander_serviceAction.h>

/*
double frequency;
double amp;
double num_cycles;
*/

ros::Time begin_time;
ros::Time end_time;
double elapsed_time;


void doneCb(const actionlib::SimpleClientGoalState& state, const commander_ros_action::commander_serviceResultConstPtr& result) {
    end_time = ros::Time::now();
    elapsed_time = -begin_time.toSec() + end_time.toSec();
    ROS_INFO("Elapsed time: %f \n", elapsed_time);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "commander_action_client_node"); //Name the node 

    commander_ros_action::commander_serviceGoal wave_inputs; //Create an object of the class

    //Creating client node with name wave_input_client
    actionlib::SimpleActionClient<commander_ros_action::commander_serviceAction> action_client("wave_input", true);

    //Comment out bottom lines for debugging
    ROS_INFO("Waiting for server: ");

    bool server_exists = action_client.waitForServer(ros::Duration(5.0));

    if(!server_exists) {
        ROS_WARN("Could not connect to server; halting");
        return 0;
    }
    
    ROS_INFO("Connected to action server");

    while(true) {

        cout << "Enter frequency: " ;
        cin >> wave_inputs.frequency;
        cout << endl << "Enter amplitude: ";
        cin >> wave_inputs.amplitude;
        cout << endl << "enter number of cycles: ";
        cin >> wave_inputs.cycles;

        //Time when goal is sent to service
        begin_time = ros::Time::now();
        action_client.sendGoal(wave_inputs, &doneCb);
    }

}