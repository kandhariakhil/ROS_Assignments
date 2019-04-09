//irb120_dance_commander
//Akhil Kandhari
//axk751@case.edu
//Reference sin_commander_traj for irb120

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectory.h>

//After looking into the topics being published, the code needs to publish to different joint commands
//For example publish a command to irb120/joint1_position_controller/command
//These expect a message type of std_msgs/Float64 type as included above.


int main(int argc, char** argv) {
    ros::init(argc, argv, "dance_commander"); // name this node 

    ros::NodeHandle nh; //standard ros node handle   

    //irb120 control interface uses this topic to receive trajectories
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    trajectory_msgs::JointTrajectory dance_trajectory; //trajectory message by the name of dance_trajectory
    //Point to populate the trajectory message
    trajectory_msgs::JointTrajectoryPoint trajectory_point0, trajectory_point1, trajectory_point2, trajectory_point3, trajectory_point4, trajectory_point5, trajectory_point6;

    //Fill in all the joint names
    dance_trajectory.points.clear();
    dance_trajectory.joint_names.push_back("joint1");
    dance_trajectory.joint_names.push_back("joint2");
    dance_trajectory.joint_names.push_back("joint3");
    dance_trajectory.joint_names.push_back("joint4");
    dance_trajectory.joint_names.push_back("joint5");
    dance_trajectory.joint_names.push_back("joint6");

    ros::Rate sleep_timer(1.0); //1Hz update rate
    
    // build an example trajectory:
    trajectory_point0.positions.clear();
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear();  

    //specify start pose (all zeros) and two points:
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point0.positions.push_back(0.0); //all zeros
        trajectory_point1.positions.push_back(0.0); // 

    }
    //specify arrival time (from start of trajectory)
    trajectory_point0.time_from_start = ros::Duration(1.0); //1 second to arrive

    dance_trajectory.points.clear();
    dance_trajectory.header.stamp = ros::Time::now();
    dance_trajectory.points.push_back(trajectory_point0);

    ROS_INFO("Sending robot to home position: ");
    int npts = dance_trajectory.points.size();
    ROS_INFO("Number of points in initial trajectory = %d",npts);

        for (int i=0;i<3;i++) //send this twice, just to make sure have communication w/ subscriber
        {
            pub.publish(dance_trajectory);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }

    //Command to send is listed below:

  
    double amp_joint2 = M_PI/4;
    double amp_joint3 = M_PI/8;
    double omega = 3.0;

    double dt = 0.1;
    double Tfinal = 30.0;
    double t = 0;
    int n_traj_pts = Tfinal/dt;

    dance_trajectory.points.clear();

    for (int pt=0;pt<n_traj_pts;pt++) 
    {
        trajectory_point1.positions[0] = omega*t;
        trajectory_point1.positions[1] = amp_joint2*sin(omega*t);
        trajectory_point1.positions[2] = amp_joint3*sin(omega*t);
        trajectory_point1.positions[3] = omega*t;
        trajectory_point1.positions[4] = omega*t;
        trajectory_point1.positions[5] = omega*t;
        trajectory_point1.time_from_start = ros::Duration(t);
        t+=dt;

        //Package containing different points for the arm
        dance_trajectory.points.push_back(trajectory_point1);
    }

    dance_trajectory.header.stamp = ros::Time::now();

    npts = dance_trajectory.points.size();

    int njts = dance_trajectory.points[0].positions.size();
    ROS_INFO("Sending a trajectory with %d poses, each with %d joint",npts,njts);
    pub.publish(dance_trajectory);

     for (int i=0;i<3;i++) 
     {
            sleep_timer.sleep();
     }

    return 0;
}    
