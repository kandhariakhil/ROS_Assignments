#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<math.h>

#include<commander_ros_action/commander_serviceAction.h>

using namespace std;

double vel_cmd;

class CommanderActionServer {
private:

    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<commander_ros_action::commander_serviceAction> as_;

    commander_ros_action::commander_serviceGoal inputs_; // goal message, received from client
    commander_ros_action::commander_serviceResult results_; // put results here, to be sent back to the client when done w/ goal
    commander_ros_action::commander_serviceFeedback progress_; // for feedback 

public:
    CommanderActionServer();

    ~CommanderActionServer(void){    
    }

    void executeCB(const actionlib::SimpleActionServer<commander_ros_action::commander_serviceAction>::GoalConstPtr& goal);
};

CommanderActionServer::CommanderActionServer() : as_(nh_, "wave_input", boost::bind(&CommanderActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

void CommanderActionServer::executeCB(const actionlib::SimpleActionServer<commander_ros_action::commander_serviceAction>::GoalConstPtr& goal) {

    ros::Rate timer(100);
    float dt = 0.01;
    float runtime = 0;

    int total_cycle_num = (goal->cycles/goal->frequency)/dt;

    //cout not working here?
    while(total_cycle_num > 0) {
        vel_cmd = goal->amplitude*sin(2*3.14*goal->frequency*runtime);
        runtime = runtime+dt;
        total_cycle_num = total_cycle_num-1;
        ROS_INFO("Velocity: %f \n", vel_cmd);
        timer.sleep();
    }

    results_.result = "Results have been updated";
    as_.setSucceeded(results_);
    
}

int main(int argc, char **argv) {

    vel_cmd = 0;

    ros::init(argc, argv, "commander_action_server_node");
    
    ROS_INFO("instantiating the timer_action_server: ");

    CommanderActionServer as_object;

    ROS_INFO("going into spin");

    
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;

}