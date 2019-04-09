//Ariac introdcution assignment
//Akhil Kandhari, axk751@case.edu
// Created 11/4/2018

#include <ros/ros.h>
#include <std_srvs/Trigger.h> //Include service type for starting competition
#include <osrf_gear/ConveyorBeltControl.h> //Include service type to start conveyor belt
#include <osrf_gear/DroneControl.h> // Include service type to start drone collection
#include <osrf_gear/LogicalCameraImage.h> //Inlcude to get reading from camera
#include <iostream>
#include <string>


using namespace std;

bool stop_belt = false;
float dist;
// How to read z value of box? Can read z value of cam but not box!

// void myCallback(const osrf_gear::LogicalCameraImage& cam_data)
// {   
//     cout<<cam_data;
//     ROS_INFO("Receiving input from Logical Camera %f",dist);
    
//     if(dist < 0.1){
//         stop_belt = true;
//     }
//     ros::Duration(1.0).sleep();
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "Run_Ariac_Simulation");
    ros::NodeHandle n;

    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger StartupSrv;

    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl ConveyorSrv;

    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl DroneSrv;    

    startup_client.call(StartupSrv);

    while(StartupSrv.response.success == false){
        ROS_WARN("Competition start not successful");
        startup_client.call(StartupSrv);
        ros::Duration(1.0).sleep();
    }


    ConveyorSrv.request.power = 100.0;
    conveyor_client.call(ConveyorSrv);
    
    while(ConveyorSrv.response.success == false){
        ROS_WARN("Conveyor start not successful");
        conveyor_client.call(ConveyorSrv);
        ros::Duration(1.0).sleep();
    }
   
    // ros::Subscriber logical_camera_data = n.subscribe("ariac/logical_camera_2",1,myCallback);

    // if(stop_belt == false){  
    //     ros::Subscriber logical_camera_data = n.subscribe("ariac/logical_camera_2",1,myCallback);
    //     ROS_INFO("Spinning");
    //     ros::spin();
    // }
    

    ConveyorSrv.request.power = 0;
    conveyor_client.call(ConveyorSrv);

    ros::Duration(5.0).sleep();

    ConveyorSrv.request.power = 100.0;
    conveyor_client.call(ConveyorSrv);    

    DroneSrv.request.shipment_type = "order_0_shipment_0";
    drone_client.call(DroneSrv);

    while(DroneSrv.response.success == false){
        drone_client.call(DroneSrv);
        ros::Duration(1.0).sleep();
    }

 
}